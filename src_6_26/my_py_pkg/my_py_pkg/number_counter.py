#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_number=0
        self.subscriber=self.create_subscription(
            Int64,
            "number_topic",
            self.callback_number_counter, 
            10)
        self.number_counter_publisher= self.create_publisher(Int64, 
                                                             "counted_number_topic", 10)
        self.reset_counter_service = self.create_service(
            SetBool,
            'reset_counter',
            self.reset_counter_service_callback)
        self.get_logger().info("Number Counter is online.")
        
    def callback_number_counter(self, msg):
        self.counter_number += msg.data
        self.number_counter_publisher.publish(Int64(data=self.counter_number))
    def reset_counter_service_callback(self, request:SetBool.Request, response:SetBool.Response):
        if request.data:
            self.counter_number = 0
            response.success = True
            response.message = "Counter has been reset to zero."
            self.get_logger().info("Counter reset to zero via service call.")
        else:
            response.success = False
            response.message = "Counter reset not performed. Request data was False."
        return response
def main(args=None):
    rclpy.init(args=args)

    number_counter=NumberCounter()
    rclpy.spin(number_counter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()