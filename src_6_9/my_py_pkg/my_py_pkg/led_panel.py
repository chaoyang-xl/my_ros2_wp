#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStatesArray
from my_robot_interfaces.srv import SetLedState
# ros2 run my_py_pkg led_panel_node --ros-args -p led_states:=[0,1,1]
#ros2 run my_py_pkg led_panel_node --ros-args --params-file ./src/yaml_files/led_cfg.yaml 

class Led_panel(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states",[0,0,0])
        self.led_states_publisher = self.create_publisher(LedStatesArray, "led_states", 10)
        self.led_states_ = self.get_parameter("led_states").value
        self.led_states_service = self.create_service(
            SetLedState,
            "set_led_state",
            self.set_led_states_callback
        )
        self.timer = self.create_timer(3.0, self.led_states_publish)
        self.get_logger().info("LED Panel Node has been started.")
        
    def led_states_publish(self):
        msg = LedStatesArray()
        msg.led_states = self.led_states_
        self.led_states_publisher.publish(msg)

    def set_led_states_callback(self, request: SetLedState.Request, response: SetLedState.Response):
        led_number = request.led_number
        led_state = request.led_state
        if led_number < 0 or led_number >=len(self.led_states_) or led_state not in [0,1]:
            response.success = False
            return response
        else:
            self.led_states_[led_number] = led_state
            response.success = True
            return response
def main(args=None):
    rclpy.init(args=args)

    led_panel=Led_panel()
    rclpy.spin(led_panel)
    rclpy.shutdown()


if __name__ == "__main__":
    main()