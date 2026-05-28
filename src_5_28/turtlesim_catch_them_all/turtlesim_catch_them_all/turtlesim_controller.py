#!/usr/bin/env python3
import rclpy
import math
from math import atan2
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray

class TurtlesimController(Node):
    def __init__(self):
        super().__init__("turtlesim_controller")
        self.target_x = 8.0
        self.target_y = 3.0
        self.target_turtle_name = ""
        self.pose: Pose = None
        self.cmd_vel_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.reached_turtle_publisher = self.create_publisher(Turtle, "reached_turtle", 10)
        self.pose_subscription = self.create_subscription(
            Pose,
            "turtle1/pose",
            self.pose_callback,
            10,
        )
        self.alive_turtles_subscription = self.create_subscription(
            TurtleArray,
            "alive_turtles",
            self.alive_turtles_callback,
            10,
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def alive_turtles_callback(self, msg: TurtleArray):
        if len(msg.trutles) > 0:
            target_turtle = msg.trutles[0]
            self.target_x = target_turtle.x
            self.target_y = target_turtle.y
            self.target_turtle_name = target_turtle.name
    def control_loop(self):
        if self.pose is None or self.target_turtle_name == "":
            return
        twist = Twist()
        dist_x = self.target_x - self.pose.x
        dist_y = self.target_y - self.pose.y
        distance = (dist_x**2 + dist_y**2) ** 0.5
        if distance < 0.1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            #kill target turtle
            reached_turtle_msg = Turtle()
            reached_turtle_msg.x = self.target_x
            reached_turtle_msg.y = self.target_y
            reached_turtle_msg.name = self.target_turtle_name
            self.reached_turtle_publisher.publish(reached_turtle_msg)
        else:

            dx = self.target_x - self.pose.x
            dy = self.target_y - self.pose.y

            angle_to_target = math.atan2(dy, dx)
            angle_diff = angle_to_target - self.pose.theta
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            twist = Twist()

            # 角速度：只管角度
            twist.angular.z = angle_diff

            # 线速度：角度越准，走得越快
            twist.linear.x = math.cos(angle_diff)

            # 防止倒着走
            if twist.linear.x < 0.0:
                twist.linear.x = 0.0

            self.cmd_vel_publisher.publish(twist)
def main(args=None):
    rclpy.init(args=args)

    turtlesim_controller=TurtlesimController()
    rclpy.spin(turtlesim_controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
