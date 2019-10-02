import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', execute_callback=self.execute_callback, callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    async def execute_callback(self, goal_handle):
        self.get_logger().info('executing...')
        msg = Fibonacci.Feedback()
        msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('goal_canceled')
                return Fibonacci.Result()

            msg.sequence.append(msg.sequence[i] + msg.sequence[i-1])
            self.get_logger().info('feedback:{}'.format(msg.sequence))
            goal_handle.publish_feedback(msg)
            time.sleep(1)  # dummy job

        goal_handle.set_succeeded()

        result = Fibonacci.Result()
        result.sequence = msg.sequence
        self.get_logger().info('result:{}'.format(result.sequence))
        return result


def main(args=None):
    rclpy.init(args=args)
    minimal_action_server = MinimalActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(minimal_action_server, executor=executor)
    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
