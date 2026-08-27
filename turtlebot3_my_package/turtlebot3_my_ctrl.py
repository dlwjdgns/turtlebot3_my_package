import rclpy as rp
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

is_sim = True

class PID:
    def __init__(self, P, I, D):
        self.p = P
        self.i = I
        self.d = D
        self.previous_err = 0.0
        self.integral = 0.0

    def update(self, error):
        self.integral += error
        derivative = error - self.previous_err
        self.previous_err = error
        return (self.p * error) + (self.i * self.integral) + (self.d * derivative)

    def reset(self):
        self.previous_err = 0.0
        self.integral = 0.0

def normalize_angle(angle):
    """각도를 -pi ~ pi로 정규화"""
    return math.atan2(math.sin(angle), math.cos(angle))

class SequentialNavigator(Node):
    def __init__(self):
        super().__init__("sequential_navigator")
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        #1. Nav2 순회할 목표 좌표 목록 (x, y, 도착 시 yaw 각도(degree))
        if not is_sim:
            self.waypoints_data = [
                (1.03, 1.84, 0.0),
                (0.0, 0.784, 90.0),
                (-0.373, 1.88, 180.0),
                (0.0, 0.0, 0.0)
            ]
            # 2. 정밀 주차/도킹 목표 좌표 (x, y, 목표 yaw 각도(degree))
            self.parking_goal_data = (-0.143, 0.0156, 0.0)
        else:
            self.waypoints_data = [
                # (0.5, 2.1, 0.0),
                # (3.5, 2.1, 90),
                # (3.5, -1.18, 180.0),
                (0.0, 0.0, 0.0)
            ]
            # 2. 정밀 주차/도킹 목표 좌표 (x, y, 목표 yaw 각도(degree))
            self.parking_goal_data = (0.710, 0.500, 180)
        self.current_index = 0

        self.angle_tolerance = 0.05       # 약 2.8도
        self.distance_tolerance = 0.05    # 5cm 정밀 공차

        self.angular_pid = PID(P=1.2, I=0.0, D=0.1)
        self.linear_pid = PID(P=0.3, I=0.0, D=0.05)

        # TF 리스너 및 버퍼
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz 제어 루프

        self.goal_pose = None
        self.state = "navigating_waypoints"  # navigating_waypoints -> rotate_to_goal -> move_to_goal -> rotate_to_final -> finished

    def set_parking_goal(self, x, y, degree=0.0):
        """주차 목표 위치 설정 및 PID 주차 모드 진입"""
        self.goal_pose = self.create_pose(x, y, degree)
        self.state = 'rotate_to_goal'
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.get_logger().info(f'★ 정밀 주차 모드 시작 -> 목표 (x={x}, y={y}, yaw={degree}°), state: {self.state}')

    def control_loop(self):
        """정밀 주차 상태 머신 제어 루프"""
        # ROS 컨텍스트가 종료 중이거나 비활성 상태면 즉시 리턴
        if not rp.ok() or self.state in ["navigating_waypoints", "idle", "finished"] or self.goal_pose is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rp.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception:
            return

        twist_stamp_msg = TwistStamped()
        twist_stamp_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamp_msg.header.frame_id = 'base_link'

        if self.state == "rotate_to_goal":
            twist_stamp_msg = self.handle_rotate_to_goal(current_x, current_y, current_yaw, twist_stamp_msg)
        elif self.state == "move_to_goal":
            twist_stamp_msg = self.handle_move_to_goal(current_x, current_y, current_yaw, twist_stamp_msg)
        elif self.state == "rotate_to_final":
            twist_stamp_msg = self.handle_rotate_to_final(current_yaw, twist_stamp_msg)

        # 퍼블리시 직전 컨텍스트 확인
        if rp.ok():
            self.cmd_vel_pub.publish(twist_stamp_msg)

    def handle_rotate_to_goal(self, current_x, current_y, current_yaw, msg):
        """1단계: 주차 좌표를 바라보도록 회전"""
        dx = self.goal_pose.pose.position.x - current_x
        dy = self.goal_pose.pose.position.y - current_y

        desired_heading = math.atan2(-dy, -dx) #후진 주차를 위한 부호
        error_angle = normalize_angle(desired_heading - current_yaw)

        if abs(error_angle) > self.angle_tolerance:
            msg.twist.angular.z = float(np.clip(self.angular_pid.update(error_angle), -0.5, 0.5))
        else:
            msg.twist.angular.z = 0.0
            self.state = "move_to_goal"
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.get_logger().info(f'주차 지점 방향 정렬 완료 -> 주행 시작. state: {self.state}')
        return msg

    def handle_move_to_goal(self, current_x, current_y, current_yaw, msg):
        """2단계: 주차 좌표까지 전진하면서 방향 미세 보정"""
        dx = self.goal_pose.pose.position.x - current_x
        dy = self.goal_pose.pose.position.y - current_y
        dist_err = math.sqrt(dx**2 + dy**2)

        if dist_err > self.distance_tolerance:
            msg.twist.linear.x = float(np.clip(self.linear_pid.update(dist_err), -0.15, 0.15))
            msg.twist.linear.x = -msg.twist.linear.x

            desired_heading = math.atan2(-dy, -dx) #후진 주차를 위한 부호
            error_angle = normalize_angle(desired_heading - current_yaw)
            msg.twist.angular.z = float(np.clip(self.angular_pid.update(error_angle), -0.4, 0.4))
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.state = "rotate_to_final"
            self.angular_pid.reset()
            self.get_logger().info(f'주차 좌표 도달 완료 -> 최종 방향 정렬 시작. state: {self.state}')
        return msg

    def handle_rotate_to_final(self, current_yaw, msg):
        """3단계: 최종 주차 목표 Heading(Yaw) 각도로 정렬"""
        q = self.goal_pose.pose.orientation
        _, _, final_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        final_error_angle = normalize_angle(final_yaw - current_yaw)

        if abs(final_error_angle) > self.angle_tolerance:
            msg.twist.angular.z = float(np.clip(self.angular_pid.update(final_error_angle), -0.4, 0.4))
        else:
            msg.twist.angular.z = 0.0
            self.state = "finished"
            self.goal_pose = None
            self.get_logger().info('★ 최종 정밀 주차/도킹 완료!')
            
            # 1. 정지 명령 먼저 퍼블리시
            if rp.ok():
                self.cmd_vel_pub.publish(msg)
            # 2. 타이머 중지
            self.timer.cancel()
            # 3. 종료
            rp.shutdown()
        return msg

    def yaw_degree_to_quaternion(self, degree):
        rad = np.deg2rad(degree)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = float(np.sin(rad / 2.0))
        q.w = float(np.cos(rad / 2.0))
        return q

    def create_pose(self, x, y, degree=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = self.yaw_degree_to_quaternion(degree)
        return pose

    def start_navigation(self):
        self.get_logger().info('NavigateToPose 액션 서버 연결 대기 중...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose 액션 서버를 찾을 수 없습니다.')
            rp.shutdown()
            return
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints_data):
            self.get_logger().info('모든 Nav2 웨이포인트 주행 완료! 정밀 주차 루틴을 시작합니다.')
            # 웨이포인트 순회 종료 후 최종 주차 PID 모드로 전환
            self.set_parking_goal(
                self.parking_goal_data[0],
                self.parking_goal_data[1],
                self.parking_goal_data[2]
            )
            return

        target = self.waypoints_data[self.current_index]
        goal_pose = self.create_pose(target[0], target[1], target[2])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'[{self.current_index + 1}/{len(self.waypoints_data)}] Nav2 목표 전송: x={target[0]}, y={target[1]}, yaw={target[2]}°')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'목표 지점 {self.current_index + 1} 요청 거절됨.')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'목표 지점 {self.current_index + 1} 도착 완료!')
            self.current_index += 1
            # 1초 후 다음 목표 전송 (리소스 안정화)
            self._timer = self.create_timer(1.0, self._delayed_send_next_goal)
        else:
            self.get_logger().error(f'목표 지점 {self.current_index + 1} 주행 실패 (상태 코드: {status})')
            rp.shutdown()

    def _delayed_send_next_goal(self):
        self._timer.cancel()
        self.destroy_timer(self._timer)
        self.send_next_goal()

def main(args=None):
    rp.init(args=args)
    navigator = SequentialNavigator()
    
    # Nav2 웨이포인트 순회 시작 -> 완료 후 내부에서 정밀 주차(set_parking_goal) 자동 연계
    navigator.start_navigation()

    try:
        rp.spin(navigator)
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        navigator.destroy_node()
        if rp.ok():
            rp.shutdown()

if __name__ == '__main__':
    main()