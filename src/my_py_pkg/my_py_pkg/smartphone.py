#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
class SmartPhone(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.get_logger().info("SmartPhone node has started.")
        self.subscriber=self.create_subscription(
            String,
            "robot_news",
            self.news_callback, 
            10)
    def news_callback(self, msg):
        self.get_logger().info("News received: " + msg.data)
def main(args=None):
    rclpy.init(args=args)
    smartphone_node=SmartPhone()
    rclpy.spin(smartphone_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()