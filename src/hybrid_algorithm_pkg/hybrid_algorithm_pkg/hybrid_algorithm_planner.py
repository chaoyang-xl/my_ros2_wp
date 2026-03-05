#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .hybrid_astar import Node as AStarNode, hybrid_astar, extract_path

# OccupancyGrid：用于接收地图数据（SLAM 或静态地图）。
# Path：用于发布规划出的路径。
# PoseStamped：用于表示路径中的单个位姿。
# PoseWithCovarianceStamped：用于接收 RViz2 中通过 “2D Pose Estimate” 发布的初始位姿（带协方差）。

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
import math

class HybridAlgorithmPlanner(Node):
    def __init__(self):
        super().__init__("hybrid_algorithm_planner")
        self.map_data = None # 存储地图数据
        self.start = None # 存储起点（AStarNode 对象）。
        self.goal = None # 存储目标点（AStarNode 对象）。
        self.map_resoultion = None # 存储地图分辨率
        self.map_origin_x = None # 存储地图原点 x 坐标
        self.map_origin_y = None # 存储地图原点 y 坐标
        self.create_subscription(OccupancyGrid, 
                                 "/map", 
                                 self.map_callback,
                                 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/initialpose",
                                 self.start_callback,
                                 10)
        self.create_subscription(PoseStamped,
                                 "/goal_pose",
                                 self.goal_callback,
                                 10)
        self.path_publisher = self.create_publisher(Path, "/planned_path", 10)
    
    # 从 OccupancyGrid 消息中获取地图宽度和高度以及地图数据
    # 并将其存储在 self.map_data 中。
    def map_callback(self, msg: OccupancyGrid):       
        self.get_logger().info("Map data received.")
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        self.map_data = data
        data[data < 0] = 0
        data[data > 50] = 1
        data[data <= 50] = 0
        self.map_resoultion = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y


    def start_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info("Initial pose received.")
        wx=msg.pose.pose.position.x
        wy=msg.pose.pose.position.y
        # 将世界坐标转换为地图坐标
        x = int((wx - self.map_origin_x) / self.map_resoultion)
        y = int((wy - self.map_origin_y) / self.map_resoultion)
        yaw = self.get_yaw(msg.pose.pose.orientation) # 使用 get_yaw 函数从四元数中提取偏航角 yaw
        self.start = AStarNode(x, y, yaw)

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info("Goal pose received. Starting path planning...")
        wx=msg.pose.position.x
        wy=msg.pose.position.y
        # 将世界坐标转换为地图坐标
        x = int((wx - self.map_origin_x) / self.map_resoultion)
        y = int((wy - self.map_origin_y) / self.map_resoultion)
        yaw = self.get_yaw(msg.pose.orientation) # 使用 get_yaw 函数从四元数中提取偏航角 yaw
        self.goal = AStarNode(x, y, yaw)


        self.plan()

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def plan(self):
        if self.start is None or self.goal is None:
            self.get_logger().warn("Start or goal pose not set. Cannot plan path.")
            return
        if self.map_data is None :
            self.get_logger().warn("No map data available. Cannot plan path.")
            return
        result = hybrid_astar(self.start, self.goal, self.map_data)
        if result is None:
            self.get_logger().warn("No path found.")
            return
        else:
            path_points = extract_path(result)
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            for x, y in path_points:
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = "map"
                # 将栅格坐标转换回世界坐标 (单位：米)
                pose.pose.position.x = (x * self.map_resoultion) + self.map_origin_x
                pose.pose.position.y = (y * self.map_resoultion) + self.map_origin_y
                pose.pose.position.z = 0.0
                    
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.path_publisher.publish(path_msg)
            self.get_logger().info("Path published with {} points.".format(len(path_points)))

def main(args=None):
    rclpy.init(args=args)

    hybrid_algorithm_planner=HybridAlgorithmPlanner()
    rclpy.spin(hybrid_algorithm_planner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()