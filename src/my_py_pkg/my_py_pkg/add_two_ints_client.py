#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
class Add_two_ints_client(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        self.get_logger().info("Add Two Ints Client is online.")
    def send_request(self,a,b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(self.request)
        future.add_done_callback(self.callback_add_two_ints)

    def callback_add_two_ints(self, future):
        response = future.result()
        self.get_logger().info(f'Result: {self.request.a} + {self.request.b} = {response.sum}')

def main(args=None):
    rclpy.init(args=args)

    add_two_ints_node=Add_two_ints_client()
    add_two_ints_node.send_request(7, 8)
    add_two_ints_node.send_request(1, 3)
    add_two_ints_node.send_request(4, 2)
    rclpy.spin(add_two_ints_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()