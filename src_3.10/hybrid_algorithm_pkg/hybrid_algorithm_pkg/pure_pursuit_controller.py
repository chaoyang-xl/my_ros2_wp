#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.path = []
        self.current_pose = None

        # --- 核心参数调整区 ---
        self.lookahead_distance = 0.4  # 预瞄距离 Ld (米)
        self.target_speed = 0.2        # 目标线速度 v (m/s)
        # ---------------------

        # 订阅全局规划器发布的路径
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        # 订阅 Gazebo 桥接过来的机器人真实里程计
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 发布速度指令给 Gazebo
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 控制循环，运行频率 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Pure Pursuit Controller Initialized.")

    def path_callback(self, msg):
        # 将接收到的 PoseStamped 数组简化为 (x, y) 坐标点列表
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"Received new path to track ({len(self.path)} points).")

    def odom_callback(self, msg):
        # 获取机器人的实时位置和偏航角
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.get_yaw(q)
        self.current_pose = (x, y, yaw)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # 如果没有路径或没有获取到位置，发送 0 速度确保车子静止
        if not self.path or self.current_pose is None:
            self.cmd_pub.publish(Twist())
            return

        cx, cy, cyaw = self.current_pose
        cmd = Twist()

        # 【核心修复 1】：判断是否到达终点。放宽停车容差到 0.25 米，防止“绕圈圈”死锁
        dist_to_goal = math.hypot(self.path[-1][0] - cx, self.path[-1][1] - cy)
        if dist_to_goal < 0.25:
            self.cmd_pub.publish(Twist()) # 踩死刹车
            self.path = [] # 清空规划好的路径
            self.get_logger().info("Goal Reached! Robot Stopped.")
            return

        # 【核心修复 2】：防止机器人回头找起点。先找到离机器人当前位置“最近”的路径点
        closest_idx = 0
        min_dist = float('inf')
        for i in range(len(self.path)):
            dist = math.hypot(self.path[i][0] - cx, self.path[i][1] - cy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 从最近点开始，顺着路径往后找满足 lookahead_distance 的“预瞄点”
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path)):
            dist = math.hypot(self.path[i][0] - cx, self.path[i][1] - cy)
            if dist >= self.lookahead_distance:
                target_idx = i
                break

        # 如果往后找都找不到足够远的点，说明马上就到终点了，直接死死盯住最后一个点
        if target_idx == closest_idx:
            target_idx = len(self.path) - 1

        tx, ty = self.path[target_idx]

        # 计算朝向误差 Alpha
        alpha = math.atan2(ty - cy, tx - cx) - cyaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # 核心公式发布指令
        cmd.linear.x = self.target_speed
        cmd.angular.z = (2.0 * self.target_speed * math.sin(alpha)) / self.lookahead_distance
        
        # 保护机制：限制最大角速度，防止急转弯翻车
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()