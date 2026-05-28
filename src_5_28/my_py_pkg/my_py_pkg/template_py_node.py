#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CustomerName(Node):#modify name
    def __init__(self):
        super().__init__("name")#modify name
def main(args=None):
    rclpy.init(args=args)

    customernode=CustomerName()#modify name
    rclpy.spin(customernode)#modify name
    rclpy.shutdown()


if __name__ == "__main__":
    main()