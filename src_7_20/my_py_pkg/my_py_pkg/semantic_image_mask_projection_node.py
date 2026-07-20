#!/usr/bin/env python3
"""
把一张语义 mask 或整张深度图中的像素点，批量反投影成 map 坐标系下的点云。

这个文件是一个独立示例，不会修改原来的 semantic_projection_node.py。

典型使用场景：
1. YOLO/YOLO-seg 输出某个目标的 mask。
2. 本节点订阅 mask、depth image、camera_info。
3. 对 mask 内的有效深度像素做 2D -> 3D 反投影。
4. 使用 TF 把相机坐标系下的一整批 3D 点转换到 map 坐标系。
5. 发布 map 下的 PointCloud2 点云；同时额外发布一个中心点方便调试。
"""

import math

import cv2
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as point_cloud2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)


class SemanticImageMaskProjectionNode(Node):
    def __init__(self):
        super().__init__("semantic_image_mask_projection_node")

        # ---------- 可通过 ros2 run/launch 传入的参数 ----------
        # 深度图话题。Orbbec 常见话题可能是 /camera/depth/image_raw。
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        # mask 话题。后续 YOLO-seg 可以把单个目标 mask 发布到这里。
        self.declare_parameter("mask_topic", "/semantic/mask")
        # 投影模式：
        # - "mask"：只转换 mask 内的像素，适合 YOLO-seg 的目标区域。
        # - "full_image"：转换整张深度图中的有效像素，适合先看整幅深度图投影效果。
        self.declare_parameter("projection_mode", "mask")
        # 相机内参话题。fx、fy、cx、cy 都从 CameraInfo 中读取。
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        # 输出目标坐标系。导航地图里一般使用 map。
        self.declare_parameter("target_frame", "map")
        # 如果这里为空，就使用 mask 或 depth 消息 header.frame_id。
        # 如果你的深度值 Z 是沿相机光轴向前，通常应该使用 camera_optical_link。
        self.declare_parameter("camera_frame_override", "camera_optical_link")
        # 16UC1 深度图通常单位是毫米，需要乘 0.001 变成米。
        # 如果你的深度图本身就是 32FC1 米，则这个参数不会被用到。
        self.declare_parameter("depth_scale", 0.001)
        # 过滤太近或太远的深度，避免无效点影响中心点。
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 6.0)
        # mask 内像素可能很多，按步长抽样可以降低 CPU 压力。
        self.declare_parameter("sample_step", 4)
        # mask 灰度值大于该阈值时认为属于目标。二值 mask 常用 1 或 255。
        self.declare_parameter("mask_threshold", 1)
        # 有效点太少时不发布，防止偶然噪声产生错误目标。
        self.declare_parameter("min_valid_points", 20)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.projection_mode = self.get_parameter("projection_mode").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame_override = self.get_parameter("camera_frame_override").value
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.sample_step = max(1, int(self.get_parameter("sample_step").value))
        self.mask_threshold = int(self.get_parameter("mask_threshold").value)
        self.min_valid_points = int(self.get_parameter("min_valid_points").value)

        # ---------- ROS 工具对象 ----------
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 最新数据缓存 ----------
        # 这里采用“缓存最新深度图 + 最新内参”的方式，逻辑简单，适合先跑通流程。
        # 后续工程化时可以换成 message_filters.ApproximateTimeSynchronizer 做严格同步。
        self.latest_depth_m = None
        self.latest_depth_header = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # 发布整片区域在 map 坐标系下的点云。
        # 这才是“转换整张图片/整张 mask”的主要输出。
        self.map_points_pub = self.create_publisher(
            PointCloud2,
            "/semantic/map_points",
            10,
        )

        # 额外发布一个中心点，方便你在 RViz 里快速看目标大概位置。
        self.map_center_pub = self.create_publisher(
            PointStamped,
            "/semantic/map_center",
            10,
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.mask_topic,
            self.mask_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.get_logger().info(
            "语义 image/mask 投影节点已启动："
            f"depth={self.depth_topic}, mask={self.mask_topic}, "
            f"camera_info={self.camera_info_topic}, "
            f"mode={self.projection_mode}, target_frame={self.target_frame}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        """
        读取相机内参。

        CameraInfo.k 是 3x3 相机内参矩阵，展开后为：
        [fx,  0, cx,
          0, fy, cy,
          0,  0,  1]
        """
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def depth_callback(self, msg: Image):
        """
        缓存最新深度图，并统一转换成“米”。

        常见深度图格式：
        - 16UC1：无符号 16 位整数，单位通常是毫米。
        - 32FC1：32 位浮点数，单位通常已经是米。
        """
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if msg.encoding == "16UC1":
            # 0 通常表示无效深度；这里先转成米，后续统一过滤。
            depth_m = depth_raw.astype(np.float32) * self.depth_scale
        else:
            # 32FC1 或其他已经由驱动转换成浮点的深度图。
            depth_m = depth_raw.astype(np.float32)

        self.latest_depth_m = depth_m
        self.latest_depth_header = msg.header

        # full_image 模式下，每来一帧深度图，就把整张深度图的有效像素投影成 map 点云。
        # 这个模式点数会比较多，所以 sample_step 不建议设成 1，先用 4 或 8 更稳。
        if self.projection_mode == "full_image" and self.has_ready_inputs():
            full_mask = np.ones(depth_m.shape[:2], dtype=np.uint8) * 255
            self.project_mask_or_image_to_map(full_mask, msg.header, "整张深度图")

    def mask_callback(self, msg: Image):
        """
        每收到一张 mask，就把 mask 内的像素点投影到 map。

        mask 可以来自：
        - YOLO-seg 的实例分割结果。
        - 你自己手工生成的二值图。
        - 调试阶段的固定 ROI 图。
        """
        if not self.has_ready_inputs():
            return

        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        # 如果 mask 和 depth 分辨率不同，需要缩放到 depth 分辨率。
        # 最近邻插值不会产生新的灰度类别，适合 mask。
        depth_h, depth_w = self.latest_depth_m.shape[:2]
        if mask.shape[:2] != (depth_h, depth_w):
            mask = cv2.resize(mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST)

        self.project_mask_or_image_to_map(mask, msg.header, "mask")

    def project_mask_or_image_to_map(self, mask, source_header, source_name):
        """
        把 mask 指定的像素区域批量转换到 map，并发布 PointCloud2。

        这里的 mask 可以是真实语义 mask，也可以是 full_image 模式下构造的全白 mask。
        所以“整张图片投影”和“mask 投影”本质上是同一个问题：
        选出一批像素 -> 查深度 -> 反投影到相机坐标 -> 批量 TF 到 map。
        """
        camera_points = self.mask_to_camera_points(mask)
        if len(camera_points) < self.min_valid_points:
            self.get_logger().warn(
                f"{source_name} 有效点过少：{len(camera_points)}，本帧不发布"
            )
            return

        if not source_header.frame_id:
            # 有些自定义 mask 消息可能没有 frame_id，此时用深度图的 frame_id 补上。
            source_header = self.latest_depth_header

        map_points = self.transform_camera_points_to_map(camera_points, source_header)
        if map_points is None or len(map_points) == 0:
            return

        cloud_msg = self.create_map_point_cloud_msg(map_points, source_header.stamp)
        self.map_points_pub.publish(cloud_msg)

        # 对 map 下的一整片点取中位数，得到一个稳一点的中心位置。
        center_map_xyz = np.median(map_points, axis=0)
        center_map = PointStamped()
        center_map.header = cloud_msg.header
        center_map.point.x = float(center_map_xyz[0])
        center_map.point.y = float(center_map_xyz[1])
        center_map.point.z = float(center_map_xyz[2])

        self.map_center_pub.publish(center_map)
        self.get_logger().info(
            f"发布 {source_name} map 点云：points={len(map_points)}, "
            f"center=({center_map.point.x:.2f}, "
            f"{center_map.point.y:.2f}, {center_map.point.z:.2f})"
        )

    def has_ready_inputs(self):
        """
        检查投影需要的三类数据是否齐全：深度图、深度图 header、相机内参。
        """
        if self.latest_depth_m is None or self.latest_depth_header is None:
            self.get_logger().warn("还没有收到 depth image，暂时无法投影 mask")
            return False
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warn("还没有收到 CameraInfo，暂时无法反投影像素")
            return False
        return True

    def mask_to_camera_points(self, mask):
        """
        把 mask 内像素批量反投影成相机坐标系下的 3D 点。

        对每个像素 (u, v) 和深度 z：
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        Z = z

        注意：
        - 这里得到的是相机光学坐标系 camera_optical_link 下的点。
        - 如果你的 frame_id 写成 camera_link，但深度的 Z 仍然表示“向前”，坐标轴可能会错。
        """
        # 先按空间步长抽样，减少点数。
        sampled_mask = mask[:: self.sample_step, :: self.sample_step]
        sampled_depth = self.latest_depth_m[:: self.sample_step, :: self.sample_step]

        v_small, u_small = np.where(sampled_mask >= self.mask_threshold)
        if len(u_small) == 0:
            return np.empty((0, 3), dtype=np.float32)

        # 抽样后的坐标还原回原图像素坐标。
        u = (u_small * self.sample_step).astype(np.float32)
        v = (v_small * self.sample_step).astype(np.float32)
        z = sampled_depth[v_small, u_small].astype(np.float32)

        valid = np.isfinite(z)
        valid = valid & (z >= self.min_depth_m) & (z <= self.max_depth_m)
        valid = valid & (z > 0.0)

        u = u[valid]
        v = v[valid]
        z = z[valid]

        if len(z) == 0:
            return np.empty((0, 3), dtype=np.float32)

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        return np.stack([x, y, z], axis=1).astype(np.float32)

    def transform_camera_points_to_map(self, camera_points, image_header):
        """
        批量把 Nx3 的相机坐标点转换到 target_frame。

        重点：
        - 这里不是一个点一个点调用 tf_buffer.transform。
        - 而是只查一次 TF 矩阵，然后用 numpy 一次性乘完所有点。
        - 对整张图片来说，这比循环 PointStamped 快很多。
        """
        source_frame = self.get_source_camera_frame(image_header)
        if not source_frame:
            self.get_logger().error("没有可用的相机 frame_id，无法查询 TF")
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                image_header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().error(
                f"TF 查询失败：{source_frame} -> {self.target_frame}, error={exc}"
            )
            return None

        rotation = self.quaternion_to_rotation_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )

        # 点从相机坐标转到 map 坐标：P_map = R * P_camera + t。
        return (camera_points @ rotation.T) + translation

    def get_source_camera_frame(self, image_header):
        """
        决定输入点属于哪个相机坐标系。

        深度图按光轴向前给 Z 时，通常应该使用 camera_optical_link。
        如果你已经确认消息 header.frame_id 正确，也可以把
        camera_frame_override 参数设为空字符串，让节点直接使用 header.frame_id。
        """
        if self.camera_frame_override:
            return self.camera_frame_override
        return image_header.frame_id

    def create_map_point_cloud_msg(self, map_points, stamp):
        """
        把 numpy 的 Nx3 点数组打包成 ROS2 PointCloud2。

        RViz 里可以直接添加 PointCloud2，话题选择 /semantic/map_points。
        这样你看到的是整张图或整个 mask 投影后的空间点，而不是单个中心点。
        """
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame
        return point_cloud2.create_cloud_xyz32(header, map_points.tolist())

    def quaternion_to_rotation_matrix(self, quat):
        """
        把 ROS 的四元数转换成 3x3 旋转矩阵。

        TF 返回的是平移 + 四元数；批量点变换时需要把四元数先变成矩阵。
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3, dtype=np.float32)

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float32,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticImageMaskProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
