from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting...')
        self.request = AddTwoInts.Request()

    def call_async(self):
        self.request.a = 1
        self.request.b = 2
        return self.client.call_async(self.request)


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    future = minimal_client.call_async()
    rclpy.spin_until_future_complete(minimal_client, future)

    if future.done() and future.result() is not None:
        response = future.result()
        minimal_client.get_logger().info('{} + {} = {}'.format(minimal_client.request.a,
                                                               minimal_client.request.b, response.sum))

    rclpy.shutdown()
