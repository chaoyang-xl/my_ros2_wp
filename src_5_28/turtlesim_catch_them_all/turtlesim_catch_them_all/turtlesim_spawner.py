#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from functools import partial

class TurtlesimSpawner(Node):
    def __init__(self):
        super().__init__("turtlesim_spawner")
        self.turtle_count = 0
        self.alive_turtles = []

        self.turtles_publisher = self.create_publisher(TurtleArray,"alive_turtles",10)

        self.client_ = self.create_client(Spawn, "spawn")
        self.kill_reached_trutle_client = self.create_client(Kill, "kill")
        self.reached_turtle_subscription = self.create_subscription(
            Turtle,
            "reached_turtle",
            self.reached_turtle_callback,
            10,
        )
        self.create_turtle()
        self.publish_alive_turtles()
        #self.spawn_timer = self.create_timer(2.0, self.timer_callback)

    def reached_turtle_callback(self, msg: Turtle):
        kill_request = Kill.Request()
        kill_request.name = msg.name
        while not self.kill_reached_trutle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        future = self.kill_reached_trutle_client.call_async(kill_request)
        future.add_done_callback(
            partial(self.callback_kill_reached_turtle, request=kill_request)
        )
        
    def callback_kill_reached_turtle(self, future, request:Kill.Request):
        self.get_logger().info(f"Killed turtle named: {request.name}")
        for i, turtle in enumerate(self.alive_turtles):
            if turtle.name == request.name:
                del self.alive_turtles[i]
                break
        self.create_turtle()
        self.publish_alive_turtles()
    # def timer_callback(self):
    #     self.turtle_count += 1
    #     x=random.uniform(1.0, 10.0)
    #     y=random.uniform(1.0, 10.0)
    #     theta=random.uniform(0.0, 6.28)
    #     name="turtle_"+str(self.turtle_count)
    #     self.call_spawn_service(x, y, theta, name)

    def create_turtle(self):
        self.turtle_count += 1
        x=random.uniform(1.0, 10.0)
        y=random.uniform(1.0, 10.0)
        theta=random.uniform(0.0, 6.28)
        name="turtle_"+str(self.turtle_count)
        self.call_spawn_service(x, y, theta, name)
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.trutles = self.alive_turtles
        self.turtles_publisher.publish(msg)

    def call_spawn_service(self, x, y, theta, name):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_service,request = request)
        )
    def callback_call_spawn_service(self, future , request:Spawn.Request):
        response:Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info(f"Spawned turtle named: {response.name}")
            new_turtle = Turtle()
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.name = response.name
            self.alive_turtles.append(new_turtle)
            self.publish_alive_turtles()
    
        else:
            self.get_logger().info("Failed to spawn turtle.")

def main(args=None):
    rclpy.init(args=args)

    turtlesim_spawner=TurtlesimSpawner()
    rclpy.spin(turtlesim_spawner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()