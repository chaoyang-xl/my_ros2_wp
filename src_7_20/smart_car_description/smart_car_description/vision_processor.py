import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        # 初始化节点名称
        super().__init__('vision_processor_node')
        
        # 创建订阅者：订阅RGB彩色图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        
        # 创建订阅者：订阅深度图像
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10)
        
        # 创建发布者：发布目标位置（包含深度信息）
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
            
        # 实例化 CvBridge，用于 ROS 图像和 OpenCV 图像之间的转换
        self.bridge = CvBridge()
        
        # HSV颜色范围 - 用于检测红色目标
        self.lower_red1 = np.array([0, 50, 40])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 50, 40])
        self.upper_red2 = np.array([180, 255, 255])
        
        # 图像中心点
        self.image_center_x = 320  # 640/2
        self.image_center_y = 240  # 480/2
        
        # 深度图像存储
        self.depth_image = None
        
        # 相机内参（根据fov和分辨率计算）
        self.fx = 554.25  # 焦距x (像素)
        self.fy = 554.25  # 焦距y (像素)
        self.cx = 320.0   # 主点x
        self.cy = 240.0   # 主点y
        
        self.get_logger().info('🚀 视觉处理节点已启动（带深度信息）')
        self.get_logger().info('🔍 正在检测红色目标并计算深度...')

    def depth_callback(self, msg):
        """接收深度图像"""
        try:
            # 深度图像通常是32位浮点型，单位米
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {e}')

    def image_callback(self, msg):
        try:
            # 将 ROS 传来的传感器消息转换为 OpenCV 可读的 BGR 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 目标检测
            target_info = self.detect_target(cv_image)
            
            # 绘制检测结果
            display_image = self.draw_detection(cv_image, target_info)
            
            # 发布目标位置
            if target_info['found']:
                point_msg = Point()
                cx, cy = target_info['x'], target_info['y']
                
                # 获取深度信息
                depth = self.get_depth_at(cx, cy)
                
                if depth is not None and depth > 0:
                    # 使用深度信息计算实际距离
                    point_msg.x = float(cx - self.image_center_x)  # 水平像素偏移
                    point_msg.y = float(cy - self.image_center_y)  # 垂直像素偏移
                    point_msg.z = depth  # 实际深度距离（米）
                    
                    # 可选：计算目标在相机坐标系中的3D位置
                    # X = (cx - cx) * depth / fx
                    # Y = (cy - cy) * depth / fy
                    # Z = depth
                    
                    self.get_logger().debug(f'目标深度: {depth:.2f}m, 位置: ({cx}, {cy})')
                else:
                    # 深度不可用，回退到使用面积估算
                    point_msg.x = float(cx - self.image_center_x)
                    point_msg.y = float(cy - self.image_center_y)
                    point_msg.z = float(target_info['area'] / 1000.0)  # 缩放面积值
                
                self.target_pub.publish(point_msg)
            
            # 显示RGB图像
            cv2.imshow("RGB - Target Detection", display_image)
            
            # 显示深度图像（如果有）
            if self.depth_image is not None:
                depth_display = self.visualize_depth(self.depth_image, target_info)
                cv2.imshow("Depth", depth_display)
            
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'图像处理失败: {e}')

    def visualize_depth(self, depth_image, target_info):
        """
        将深度图像可视化为彩色图像
        """
        # 将深度值归一化到0-255范围用于显示
        # 有效深度范围 0.1m - 10m
        depth_min, depth_max = 0.1, 5.0
        
        # 裁剪并归一化
        depth_clipped = np.clip(depth_image, depth_min, depth_max)
        depth_normalized = (depth_clipped - depth_min) / (depth_max - depth_min)
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)
        
        # 应用颜色映射（近处红色，远处蓝色）
        depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
        
        # 如果有目标，在深度图上标记
        if target_info['found']:
            cx, cy = target_info['x'], target_info['y']
            depth_at_target = self.get_depth_at(cx, cy)
            
            # 绘制目标位置
            cv2.circle(depth_color, (cx, cy), 8, (255, 255, 255), 2)
            cv2.circle(depth_color, (cx, cy), 3, (0, 0, 0), -1)
            
            # 显示深度值
            if depth_at_target is not None:
                text = f"{depth_at_target:.2f}m"
                cv2.putText(depth_color, text, (cx + 10, cy - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 添加深度图例
        cv2.putText(depth_color, "Near", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(depth_color, "Far", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        return depth_color

    def detect_target(self, cv_image):
        """
        使用颜色阈值检测目标
        返回目标信息字典
        """
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 创建红色掩码（处理红色的两个HSV范围）
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_info = {
            'found': False,
            'x': 0,
            'y': 0,
            'area': 0,
            'width': 0,
            'height': 0
        }
        
        if contours:
            # 找到最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # 过滤小面积噪声
            if area > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = x + w//2, y + h//2
                
                target_info = {
                    'found': True,
                    'x': cx,
                    'y': cy,
                    'area': area,
                    'width': w,
                    'height': h,
                    'bbox': (x, y, w, h)
                }
        
        return target_info

    def draw_detection(self, cv_image, target_info):
        """
        在图像上绘制检测结果
        """
        display = cv_image.copy()
        
        # 绘制中心十字线
        cv2.line(display, (self.image_center_x - 20, self.image_center_y), 
                 (self.image_center_x + 20, self.image_center_y), (0, 255, 0), 2)
        cv2.line(display, (self.image_center_x, self.image_center_y - 20), 
                 (self.image_center_x, self.image_center_y + 20), (0, 255, 0), 2)
        
        if target_info['found']:
            x, y, w, h = target_info['bbox']
            cx, cy = target_info['x'], target_info['y']
            
            # 绘制边界框
            cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            
            # 绘制从图像中心到目标中心的线
            cv2.line(display, (self.image_center_x, self.image_center_y), (cx, cy), (255, 0, 0), 2)
            
            # 显示偏移信息和深度
            offset_x = cx - self.image_center_x
            offset_y = cy - self.image_center_y
            depth = self.get_depth_at(cx, cy)
            if depth is not None and depth > 0:
                info_text = f"Offset: ({offset_x}, {offset_y}) Depth: {depth:.2f}m"
            else:
                info_text = f"Offset: ({offset_x}, {offset_y}) Area: {int(target_info['area'])}"
            cv2.putText(display, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No target detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return display

    def get_depth_at(self, x, y):
        """
        获取指定像素位置的深度值（米）
        使用周围区域的平均深度来提高稳定性
        """
        if self.depth_image is None:
            return None
        
        h, w = self.depth_image.shape
        
        # 确保坐标在有效范围内
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # 使用5x5区域平均来减少噪声
        x1, x2 = max(0, x - 2), min(w, x + 3)
        y1, y2 = max(0, y - 2), min(h, y + 3)
        
        region = self.depth_image[y1:y2, x1:x2]
        
        # 过滤无效值（0或inf）
        valid_depths = region[(region > 0.1) & (region < 10.0)]
        
        if len(valid_depths) > 0:
            return float(np.median(valid_depths))  # 使用中位数更鲁棒
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    
    try:
        rclpy.spin(node) # 保持节点运行，持续监听话题
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()