#!/usr/bin/env python3


#ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy //src/
#colcon build --packages-select my_py_pkg
import rclpy
from rclpy.node import Node

class myNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter=0
        self.get_logger().info("Hello ROS2 Python")
        self.timer=self.create_timer(1.0,self.timer_callback)
    def timer_callback(self):
        self.get_logger().info("hello"+str(self.counter))
        self.counter+=1
def main(args=None):
    rclpy.init(args=args)
    # node=Node("py_test")
    # node.get_logger().info(node.get_name())

    node=myNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()