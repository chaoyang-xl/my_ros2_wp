#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
class RobotNewsStation(Node):#modify name
    def __init__(self):
        super().__init__("robot_news_station")
        self.publisher=self.create_publisher(String,"robot_news",10)
        self.timer=self.create_timer(1.0,self.publish_news)
        self.get_logger().info("Robot News Station is online.")
        
    def publish_news(self):
        msg=String()
        msg.data="Hello from Robot News Station"
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    robot_news_station=RobotNewsStation()
    rclpy.spin(robot_news_station)
    rclpy.shutdown()


if __name__ == "__main__":
    main()