#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from enum import Enum
import sys
import tty
import termios
import select

class State(Enum):
    FORWARD = 0
    TURN = 1

class Gazebo_Control(Node):
    def __init__(self):
        super().__init__("gazebo_control")
        self.state = State.FORWARD
        self.state_start_time = self.get_current_time()
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.laser_subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.laser_callback,
            10
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)
    def laser_callback(self, msg: LaserScan):
            # 1. 获取正前方的距离 (Index 0)
            # 注意：有时候 Index 0 可能会是 'inf' (无穷大)，所以最好多取几个点平均一下
            #ront_dist = msg.ranges[0] 
            
            # 为了更稳健，你可以取车头极小范围内（比如 0度 ~ 1度）的最小值
            front_dist = min(min(msg.ranges[0:10]), min(msg.ranges[-10:]))
            
            print(f"Front Distance: {front_dist:.2f} m")

            # 2. 判断逻辑
            # 注意要处理 inf (无穷大，表示超出量程)
            if front_dist < 0.5 and front_dist != float('inf'):
                print("Obstacle ahead! (Stop!)")
                # 这里可以添加发布 cmd_vel 停车的代码
            else:
                print("Safe to move.")

    def get_current_time(self):
        sec,nano = self.get_clock().now().seconds_nanoseconds()
        return sec + nano * 1e-9
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = ord(sys.stdin.read(1))
        else:
            key = -1
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def control_loop(self):
        key = self.get_key()
        twist = Twist()
        if key == ord("w"):
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        elif key == ord("a"):
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        elif key == ord("d"):
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        elif key == ord("s"):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)



    # def control_loop(self):
    #     twist = Twist()
    #     now = self.get_current_time()
    #     elapsed = now - self.state_start_time

    #     if self.state == State.FORWARD:
    #         twist.linear.x = 1.0
    #         twist.angular.z = 0.0

    #         if elapsed > 3.0:
    #             self.state = State.TURN
    #             self.state_start_time = now

    #     elif self.state == State.TURN:
    #         twist.linear.x = 0.0
    #         twist.angular.z = math.pi / 4  # rad/s

    #         if elapsed > 2.0:
    #             self.state = State.FORWARD
    #             self.state_start_time = now

    #     self.cmd_vel_publisher.publish(twist)
def main(args=None):
    rclpy.init(args=args)

    gazebo_control=Gazebo_Control()
    print("Gazebo Control Node Started. Use WASD keys to control the robot.")
    rclpy.spin(gazebo_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()