#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisher(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)
        self.get_logger().info("Hardware Status Publisher has been started.")
    def publish_status(self):
        msg = HardwareStatus()
        msg.temperature = 75.0
        msg.is_operational=True
        msg.debug_message="no warnings" 
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    hardware_status_publisher=HardwareStatusPublisher()
    rclpy.spin(hardware_status_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()