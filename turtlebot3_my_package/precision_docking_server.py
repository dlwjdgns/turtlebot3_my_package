import math
import numpy as np
import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

from turtlebot3_msgs.action import PrecisionDock


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, p: float, i: float, d: float):
        self.p = p
        self.i = i
        self.d = d
        self.previous_err = 0.0
        self.integral = 0.0

    def update(self, error: float) -> float:
        self.integral += error
        derivative = error - self.previous_err
        self.previous_err = error
        return (self.p * error) + (self.i * self.integral) + (self.d * derivative)

    def reset(self):
        self.previous_err = 0.0
        self.integral = 0.0


class PrecisionDockingServer(Node):
    def __init__(self):
        super().__init__('precision_docking_server')

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            PrecisionDock,
            'precision_dock',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TwistStamped 퍼블리셔 (/cmd_vel)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # PID 제어기 설정
        self.angular_pid = PID(p=1.2, i=0.0, d=0.1)
        self.linear_pid = PID(p=0.3, i=0.0, d=0.05)
        self.angle_tolerance = 0.05       # rad (~2.8도)
        self.distance_tolerance = 0.05    # meter (5cm)

        self.get_logger().info('Precision Docking Action Server (TwistStamped) 준비 완료.')

    def goal_callback(self, goal_request):
        self.get_logger().info('★ 도킹 액션 요청 수신 -> 수락')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('도킹 액션 취소 요청 수신')
        return CancelResponse.ACCEPT

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('🚀 정밀 도킹 제어 루틴 시작')
        self.angular_pid.reset()
        self.linear_pid.reset()

        target_pose = goal_handle.request.target_pose
        goal_x = target_pose.pose.position.x
        goal_y = target_pose.pose.position.y
        q = target_pose.pose.orientation
        _, _, final_target_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        state = "rotate_to_goal"
        feedback_msg = PrecisionDock.Feedback()
        rate = self.create_rate(20.0)  # 20Hz 제어 주기

        while rp.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_stop()
                self.get_logger().warn('도킹 시퀀스 취소됨')
                return PrecisionDock.Result(success=False, message="Canceled by client")

            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                curr_x = trans.transform.translation.x
                curr_y = trans.transform.translation.y
                _, _, curr_yaw = tf_transformations.euler_from_quaternion([
                    trans.transform.rotation.x, trans.transform.rotation.y,
                    trans.transform.rotation.z, trans.transform.rotation.w
                ])
            except Exception:
                rate.sleep()
                continue

            dx = goal_x - curr_x
            dy = goal_y - curr_y
            dist_err = math.hypot(dx, dy)
            desired_heading = math.atan2(-dy, -dx)  # 후진 주차 기준 Heading
            heading_err = normalize_angle(desired_heading - curr_yaw)
            final_yaw_err = normalize_angle(final_target_yaw - curr_yaw)

            feedback_msg.distance_remaining = float(dist_err)
            feedback_msg.angle_remaining = float(final_yaw_err if state == "rotate_to_final" else heading_err)
            goal_handle.publish_feedback(feedback_msg)

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'

            if state == "rotate_to_goal":
                if abs(heading_err) > self.angle_tolerance:
                    twist_msg.twist.angular.z = float(np.clip(self.angular_pid.update(heading_err), -0.5, 0.5))
                else:
                    state = "move_to_goal"
                    self.linear_pid.reset()
                    self.angular_pid.reset()
                    self.get_logger().info('1단계 완료: 도킹 방향 정렬 -> 후진 이동 시작')

            elif state == "move_to_goal":
                if dist_err > self.distance_tolerance:
                    v_cmd = np.clip(self.linear_pid.update(dist_err), -0.15, 0.15)
                    twist_msg.twist.linear.x = float(-v_cmd)  # 후진 이동
                    twist_msg.twist.angular.z = float(np.clip(self.angular_pid.update(heading_err), -0.4, 0.4))
                else:
                    state = "rotate_to_final"
                    self.angular_pid.reset()
                    self.get_logger().info('2단계 완료: 도킹 위치 도달 -> 최종 Heading 정렬 시작')

            elif state == "rotate_to_final":
                if abs(final_yaw_err) > self.angle_tolerance:
                    twist_msg.twist.angular.z = float(np.clip(self.angular_pid.update(final_yaw_err), -0.4, 0.4))
                else:
                    self.publish_stop()
                    goal_handle.succeed()
                    self.get_logger().info('★ 최종 정밀 도킹 완료!')
                    return PrecisionDock.Result(success=True, message="Successfully docked")

            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()


def main(args=None):
    rp.init(args=args)
    server_node = PrecisionDockingServer()

    # MultiThreadedExecutor로 블로킹 방지
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        if rp.ok():
            rp.shutdown()


if __name__ == '__main__':
    main()