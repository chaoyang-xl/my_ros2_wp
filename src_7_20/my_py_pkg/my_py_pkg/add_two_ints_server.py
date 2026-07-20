#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
class Add_two_ints(Node):
    def __init__(self):
        super().__init__("add_two_ints")
        self.server=self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)
        self.get_logger().info("Add Two Ints Service is online.")
    def add_two_ints_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Request: {request.a} + {request.b} = {response.sum}")
        return response
def main(args=None):
    rclpy.init(args=args)

    add_two_ints_node=Add_two_ints()
    rclpy.spin(add_two_ints_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()