#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from rclpy.time import Time

class SemanticProjectionNode(Node):
    def __init__(self):
        super().__init__("semantic_projection_node")
        self.get_logger().info("Semantic Projection Node is online.")
        # 1. 实例化 TF Buffer，用来缓存所有的坐标变换历史（默认缓存 10 秒）
        self.tf_buffer = Buffer()
        
        # 2. 实例化监听器，它会自动订阅 /tf 和 /tf_static，并把数据填入 Buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("语义投影节点已启动，等待 TF 树就绪...")

        #测试 每2s发布一次转换结果(1.0, 1.0, 1.0)这个点在相机坐标系下的地图坐标
        self.map_point_publisher = self.create_publisher(PointStamped, 'map_point', 10)
        self.tf_publisher = self.create_publisher(TransformStamped, 'tf_transform', 10)
        self.timer = self.create_timer(2.0, self.test_transform)


    def test_transform(self):
        # 这里我们假设在相机坐标系下有一个点 (1.0, 1.0, 1.0)，我们想知道它在地图坐标系下是什么位置
        cam_x, cam_y, cam_z = 1.0, 1.0, 1.0
        
        # 创建一个假的 Header，时间戳使用当前时间，frame_id 使用相机的 link 名字
        from std_msgs.msg import Header
        image_header = Header()
        image_header.stamp = Time(seconds=0.0).to_msg() # 当前时间戳
        image_header.frame_id = 'camera_link' # 或者 'front_camera_link' 
        
        temp=self.transform_camera_point_to_map(cam_x, cam_y, cam_z, image_header)
        if temp is not None:
            self.map_point_publisher.publish(temp)
        self.get_logger().info("发布了一个地图坐标点")

    def transform_camera_point_to_map(self, cam_x, cam_y, cam_z, image_header):
        """
        将相机坐标系下的点转换到全局 map 坐标系
        :param cam_x, cam_y, cam_z: 在相机坐标系下的 3D 坐标
        :param image_header: 对应那帧相机的 Header（包含了关键的时间戳 stamp）
        """
        # 1. 组装输入点（打上时间戳和坐标系标签）
        point_in_camera = PointStamped()
        # 必须使用当时拍下这张照片那一瞬间的时间戳，而不是现在的系统时间！
        point_in_camera.header.stamp = image_header.stamp 
        # 必须指明这个点当前属于哪个坐标系（你的相机 link 名字）
        # 如果传入的深度值 Z 是朝前的，那么你的 header.frame_id 必须填 camera_optical_link 。
        # TF 树会自动帮你把坐标轴扭转过来再投影到地图上。如果不注意这点，投影出来的物体全会在天上或者地下。
        point_in_camera.header.frame_id ='camera_optical_link'     #'camera_link' 或者 'front_camera_link' 
        
        # 填入你的计算结果
        point_in_camera.point.x = float(cam_x)
        point_in_camera.point.y = float(cam_y)
        point_in_camera.point.z = float(cam_z)

        # 2. 呼叫 TF Buffer 进行时空转换
        try:
            # transform 函数会自动寻找从 'camera_link' 到 'map' 的矩阵并相乘
            # timeout 表示如果当前没查到，最多等 0.1 秒
            point_in_map = self.tf_buffer.transform(
                point_in_camera, 
                'map', 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            transform=self.tf_buffer.lookup_transform(
                'map', 
                'camera_optical_link', 
                image_header.stamp, 
                timeout=rclpy.duration.Duration(seconds=0.1)
                )
        
            self.tf_publisher.publish(transform)
            self.get_logger().info(
                f"转换成功！\n"
                f"相机坐标: ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}) -> \n"
                f"地图坐标: ({point_in_map.point.x:.2f}, {point_in_map.point.y:.2f}, {point_in_map.point.z:.2f})"
            )
            
            return point_in_map# 返回转换后的 PointStamped 对象

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # 如果 TF 树断了，或者时间戳查不到，这里会捕获异常，防止节点崩溃
            self.get_logger().error(f"坐标转换失败: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)

    semantic_projection_node=SemanticProjectionNode()
    rclpy.spin(semantic_projection_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()