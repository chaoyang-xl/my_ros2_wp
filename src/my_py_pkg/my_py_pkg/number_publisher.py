#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

#ros2 run my_py_pkg number_publisher_node --ros-args -p number:=7 -p time_period:=0.5
class NumberPublisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number" , 2)
        self.declare_parameter("time_period" , 1.0)
        self.number = self.get_parameter("number").value
        self.time_period = self.get_parameter("time_period").value
        self.number_publisher= self.create_publisher(Int64, "number_topic", 10)
        self.timer = self.create_timer(self.time_period, self.publish_number)
        self.get_logger().info("Number Publisher is online.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number 
        self.number_publisher.publish(msg)
        #self.get_logger().info(f"Published number: {msg.data}")
def main(args=None):
    rclpy.init(args=args)

    number_publisher=NumberPublisher()
    rclpy.spin(number_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()