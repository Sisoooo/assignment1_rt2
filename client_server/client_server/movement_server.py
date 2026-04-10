import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from actions_env.action import Movement


class MovementServer(Node):

    def __init__(self):
        super().__init__('movement_server')
        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Movement,
            'movement',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._current_x = 0.0
        self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10,
            callback_group=self._cb_group,
        )

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def _odom_callback(self, msg):
        self._current_x = msg.pose.pose.position.x

    def execute_callback(self, goal_handle):
        desired_x = goal_handle.request.desired_position
        self.get_logger().info(f'Goal received: x = {desired_x:.2f}')

        feedback_msg = Movement.Feedback()
        cmd = Twist()
        rate = self.create_rate(10)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Cancel requested, stopping execution')
                cmd.linear.x = 0.0
                self._cmd_pub.publish(cmd)
                goal_handle.canceled()
                result = Movement.Result()
                result.result = self._current_x
                self.get_logger().info(f'Goal canceled at x = {self._current_x:.2f}')
                return result

            remaining = desired_x - self._current_x
            feedback_msg.remaining = remaining
            goal_handle.publish_feedback(feedback_msg)

            if abs(remaining) < 0.1:
                break

            # Proportional speed, clamped to [-1.0, 1.0]
            speed = max(-1.0, min(1.0, 0.5 * remaining))
            cmd.linear.x = speed
            self._cmd_pub.publish(cmd)

            rate.sleep()

        # Stop the robot
        cmd.linear.x = 0.0
        self._cmd_pub.publish(cmd)

        goal_handle.succeed()
        result = Movement.Result()
        result.result = self._current_x
        self.get_logger().info(f'Goal reached at x = {self._current_x:.2f}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MovementServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()