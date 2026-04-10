import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from actions_env.action import Movement


class MovementClient(Node):
    def __init__(self):
        super().__init__('movement_client')
        self._action_client = ActionClient(self, Movement, 'movement')
        self._goal_handle = None
        self._cancel_sent = False

    def send_goal(self, desired_x):
        goal_msg = Movement.Goal()
        goal_msg.desired_position = desired_x
        self._desired_x = desired_x

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self._goal_handle = None
        self._cancel_sent = False

        self.get_logger().info(f'Sending goal: x = {desired_x:.2f}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.remaining
        self.get_logger().info(f'Remaining distance: {remaining:.2f}')

        if self._cancel_sent or self._goal_handle is None:
            return

        if self._desired_x > 10.0 or self._desired_x < -10.0:
            self._cancel_sent = True
            self.get_logger().warn('Goal position is outside [-10, 10], cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        resp = future.result()
        if len(resp.goals_canceling) > 0:
            self.get_logger().info('Cancel request accepted')
        else:
            self.get_logger().warn('Cancel request rejected / goal already finished')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final position: x = {result.result:.2f}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = MovementClient()

    desired_x = float(input('Enter desired x position: '))
    client.send_goal(desired_x)

    rclpy.spin(client)


if __name__ == '__main__':
    main()