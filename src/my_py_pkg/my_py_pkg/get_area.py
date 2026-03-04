#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ComputeRectangeArea

class RectangeArea(Node):
    def __init__(self):
        super().__init__("name")
        self.server=self.create_service(ComputeRectangeArea, "compute_rectangle_area", self.compute_rectangle_area_callback)
        self.get_logger().info("Compute Rectangle Area Service is online.")
    def compute_rectangle_area_callback(self, request: ComputeRectangeArea.Request, response: ComputeRectangeArea.Response):
        response.area = request.length * request.width
        self.get_logger().info(f"Request: length={request.length}, width={request.width} => area={response.area}")
        return response
def main(args=None):
    rclpy.init(args=args)

    rectange_area_node=RectangeArea()
    rclpy.spin(rectange_area_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()