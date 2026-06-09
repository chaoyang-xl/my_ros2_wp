#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLedState

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.last_check_time = self.get_current_time()
        self.battery_status = "full"
        self.client_=self.create_client(SetLedState,"set_led_state")
        self.timer_ = self.create_timer(0.1, self.battery_check)
        self.get_logger().info("Battery Node has been started.")
    def get_current_time(self):
        sec,nano = self.get_clock().now().seconds_nanoseconds()
        return sec + nano * 1e-9
    
    def battery_check(self):
        current_time = self.get_current_time()
        if self.battery_status == "full":
            if current_time - self.last_check_time >= 5.0:
                self.get_logger().info("Battery low! Turning off LED.")
                self.battery_status = "empty"
                self.send_battery_status(2, 0)
                self.last_check_time = current_time
        if self.battery_status == "empty":
            if current_time - self.last_check_time >= 6.0:
                self.get_logger().info("Battery full! Turning on LED.")
                self.battery_status = "full"
                self.send_battery_status(2, 1)
                self.last_check_time = current_time


    def send_battery_status(self,led_number,led_state):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        request = SetLedState.Request()
        request.led_number = led_number
        request.led_state = led_state
        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_set_led_state)
    def callback_set_led_state(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info("LED state set successfully.")
        else:
            self.get_logger().error("Failed to set LED state.")

def main(args=None):
    rclpy.init(args=args)

    battery_node=BatteryNode()
    rclpy.spin(battery_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()