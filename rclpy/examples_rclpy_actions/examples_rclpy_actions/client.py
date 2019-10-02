from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('feedback:{}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        status = future.result().action_status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('result:{}'.format(future.result().requence))

        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info('waiting...')
        self._action_client.wait_for_server()
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)
    action_client.destroy_node()


if __name__ == '__main__':
    main()
