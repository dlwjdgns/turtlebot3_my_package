import rclpy as rp
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Quaternion, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from turtlebot3_msgs.action import PrecisionDock


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.dock_client = ActionClient(self, PrecisionDock, 'precision_dock')

        # 1. 일반 순찰 웨이포인트 목록 (x, y, yaw_deg)
        self.waypoints = [
            (0.5, 2.1, 0.0),
            (3.5, 2.1, 90.0),
            (3.5, -1.18, 180.0),
            (0.0, 0.0, 0.0)
        ]
        self.current_wp_idx = 0

        # 2. 도킹 대기점(Staging Point) 및 최종 정밀 도킹 목표
        self.staging_pose_data = (0.0, 0.0, 0.0)         # 1단계: Nav2로 복귀할 사전 대기점
        self.dock_pose_data = (0.710, 0.500, 180.0)      # 2단계: PID 정밀 도킹할 최종 위치

        # 상태 제어 변수
        self.is_paused = False
        self.mission_state = "PATROL"  # "PATROL" -> "GO_TO_STAGING" -> "DOCKING" -> "FINISHED"
        self.nav_goal_handle = None
        self.dock_goal_handle = None
        self._delay_timer = None
        self._staging_timer = None

        # 제어 인터페이스 (Services & Topic)
        self.create_service(Trigger, '~/pause', self.srv_pause_callback)
        self.create_service(Trigger, '~/resume', self.srv_resume_callback)
        self.create_service(Trigger, '~/dock_and_exit', self.srv_dock_callback)
        self.create_subscription(String, '/patrol_cmd', self.topic_cmd_callback, 10)

        self.get_logger().info('Patrol Manager 초기화 완료 (Staging-Docking 시퀀스 적용).')

    def create_pose(self, x: float, y: float, deg: float) -> PoseStamped:
        rad = np.deg2rad(deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=float(np.sin(rad / 2.0)),
            w=float(np.cos(rad / 2.0))
        )
        return pose

    # ------------------ 제어 명령 인터페이스 ------------------
    def topic_cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "pause":
            self.pause()
        elif cmd in ["resume", "start"]:
            self.resume()
        elif cmd in ["dock", "exit"]:
            self.start_docking_sequence()

    def srv_pause_callback(self, req, res):
        res.success, res.message = self.pause()
        return res

    def srv_resume_callback(self, req, res):
        res.success, res.message = self.resume()
        return res

    def srv_dock_callback(self, req, res):
        res.success, res.message = self.start_docking_sequence()
        return res

    def pause(self):
        if self.is_paused:
            return False, "이미 일시정지 상태입니다."
        self.is_paused = True
        self.get_logger().warn('⏸️ 순찰 주행 일시정지')
        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
        if self._delay_timer:
            self.destroy_timer(self._delay_timer)
            self._delay_timer = None
        return True, "Paused"

    def resume(self):
        if not self.is_paused:
            return False, "일시정지 상태가 아닙니다."
        self.is_paused = False
        self.get_logger().info('▶️ 주행 재개')
        if self.mission_state == "PATROL":
            self.dispatch_nav_goal()
        elif self.mission_state == "GO_TO_STAGING":
            self._send_staging_nav_goal()
        return True, "Resumed"

    # ------------------ Nav2 순찰 로직 ------------------
    def start_patrol(self):
        self.get_logger().info('Nav2 및 도킹 액션 서버 대기 중...')
        self.nav_client.wait_for_server()
        self.dock_client.wait_for_server()
        self.dispatch_nav_goal()

    def dispatch_nav_goal(self):
        if self.is_paused or self.mission_state != "PATROL":
            return

        target = self.waypoints[self.current_wp_idx]
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(target[0], target[1], target[2])

        self.get_logger().info(f'📍 [순찰] 웨이포인트 [{self.current_wp_idx + 1}/{len(self.waypoints)}] 전송 -> x:{target[0]}, y:{target[1]}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response)

    def nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 목표가 거절되었습니다.')
            return
        self.nav_goal_handle = handle
        res_future = handle.get_result_async()
        res_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.nav_goal_handle = None
        status = future.result().status

        # 1. 취소된 목표는 에러가 아니므로 정상 리턴
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('이전 주행 목표 취소 처리가 완료되었습니다.')
            return

        # 2. 도킹 대기점(Staging Point) 도착 여부 확인
        if self.mission_state == "GO_TO_STAGING":
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('🏁 도킹 대기점 (0.0, 0.0) 도착 완료! 2단계 정밀 도킹을 시작합니다.')
                self._send_dock_goal()
            else:
                self.get_logger().error(f'도킹 대기점 이동 실패 (Status: {status})')
            return

        # 3. 일반 순찰 주행 완료 확인
        if self.is_paused or self.mission_state != "PATROL":
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'웨이포인트 {self.current_wp_idx + 1} 도착 완료')
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            self._delay_timer = self.create_timer(1.0, self._on_delay_timeout)
        else:
            self.get_logger().error(f'주행 실패 (Status: {status}), 다음 지점으로 건너뜁니다.')
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            self._delay_timer = self.create_timer(2.0, self._on_delay_timeout)

    def _on_delay_timeout(self):
        if self._delay_timer:
            self.destroy_timer(self._delay_timer)
            self._delay_timer = None
        self.dispatch_nav_goal()

    # ------------------ 단계별 도킹 시퀀스 (Staging -> Docking) ------------------
    def start_docking_sequence(self):
        if self.mission_state in ["GO_TO_STAGING", "DOCKING", "FINISHED"]:
            return False, "이미 도킹 시퀀스가 진행 중입니다."

        self.get_logger().info('🛑 순찰 중단 -> [1단계] 도킹 대기 위치(0.0, 0.0)로 이동합니다.')
        self.is_paused = False

        if self._delay_timer:
            self.destroy_timer(self._delay_timer)
            self._delay_timer = None

        # 진행 중인 Nav2 목표가 있다면 확실히 취소 후 대기점으로 이동
        if self.nav_goal_handle is not None:
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_patrol_goal_canceled)
        else:
            self.mission_state = "GO_TO_STAGING"
            self._send_staging_nav_goal()

        return True, "Docking sequence initiated."

    def _on_patrol_goal_canceled(self, future):
        """이전 목표 취소 완료 콜백"""
        self.mission_state = "GO_TO_STAGING"
        # Nav2 내부 액션 리소스 정리를 위해 짧은 지연 후 전송
        self._staging_timer = self.create_timer(0.2, self._delayed_send_staging)

    def _delayed_send_staging(self):
        if self._staging_timer:
            self.destroy_timer(self._staging_timer)
            self._staging_timer = None
        self._send_staging_nav_goal()

    def _send_staging_nav_goal(self):
        """1단계: Nav2로 대기 위치(0.0, 0.0) 이동"""
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(*self.staging_pose_data)
        self.get_logger().info(f'🚀 [1단계] Staging 대기점 전송: x={self.staging_pose_data[0]}, y={self.staging_pose_data[1]}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response)

    def _send_dock_goal(self):
        """2단계: PID 정밀 도킹 액션 서버 호출"""
        self.mission_state = "DOCKING"
        goal = PrecisionDock.Goal()
        goal.target_pose = self.create_pose(*self.dock_pose_data)

        self.get_logger().info(f'🎯 [2단계] 정밀 도킹 시작: x={self.dock_pose_data[0]}, y={self.dock_pose_data[1]}')
        future = self.dock_client.send_goal_async(goal)
        future.add_done_callback(self.dock_goal_response)

    def dock_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('도킹 서버가 목표를 거절했습니다.')
            return
        self.dock_goal_handle = handle
        res_future = handle.get_result_async()
        res_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.mission_state = "FINISHED"
            self.get_logger().info(f'★ 정밀 도킹 완료: {result.message} -> 노드를 종료합니다.')
        else:
            self.get_logger().warn(f'도킹 실패: {result.message}')
        rp.shutdown()


def main(args=None):
    rp.init(args=args)
    manager = PatrolManager()
    manager.start_patrol()
    try:
        rp.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        if rp.ok():
            rp.shutdown()


if __name__ == '__main__':
    main()