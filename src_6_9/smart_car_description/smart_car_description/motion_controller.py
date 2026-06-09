import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math

class MotionController(Node):
    """
    运动控制节点 - 基于视觉反馈实现目标跟踪
    使用PID控制算法控制小车运动
    """
    
    def __init__(self):
        super().__init__('motion_controller_node')
        
        # 订阅目标位置
        self.target_sub = self.create_subscription(
            Point,
            '/target_position',
            self.target_callback,
            10)
        
        # 订阅里程计信息（可选，用于更精确的控制）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # 发布速度指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器，用于持续发布速度指令
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # 目标信息
        self.target_x = 0.0  # 目标相对于图像中心的x偏移（像素）
        self.target_y = 0.0  # 目标相对于图像中心的y偏移（像素）
        self.target_depth = 0.0  # 目标深度距离（米）
        self.target_area = 0.0  # 目标面积（备用）
        self.target_detected = False
        self.target_lost_time = 0.0
        self.has_depth_info = False  # 是否收到深度信息
        
        # PID参数 - 角度控制（用于转向）
        self.kp_angle = 0.003  # 比例系数
        self.ki_angle = 0.0001  # 积分系数
        self.kd_angle = 0.001  # 微分系数
        
        # PID参数 - 距离控制（用于前进/后退）- 基于深度
        self.kp_dist = 0.5   # 深度控制比例系数（增大以使用米为单位）
        self.ki_dist = 0.01
        self.kd_dist = 0.05
        
        # 目标深度设定值（米）- 小车与目标保持的距离
        self.target_depth_setpoint = 1.5  # 保持1.5米距离
        
        # 深度有效性阈值
        self.min_valid_depth = 0.3  # 最小有效深度（米）
        self.max_valid_depth = 8.0  # 最大有效深度（米）
        
        # PID状态变量
        self.error_angle_prev = 0.0
        self.error_angle_integral = 0.0
        self.error_dist_prev = 0.0
        self.error_dist_integral = 0.0
        
        # 速度限制
        self.max_linear_speed = 0.3   # 最大线速度 (m/s)
        self.max_angular_speed = 1.0  # 最大角速度 (rad/s)
        
        # 目标丢失超时时间（秒）
        self.target_timeout = 1.0
        
        self.get_logger().info('🚗 运动控制节点已启动')
        self.get_logger().info('📍 等待目标位置信息...')
    
    def target_callback(self, msg):
        """接收目标位置信息
        msg.x: 水平像素偏移
        msg.y: 垂直像素偏移  
        msg.z: 深度距离（米）或面积（备用）
        """
        self.target_x = msg.x
        self.target_y = msg.y
        
        # 判断msg.z是深度还是面积
        # 深度值通常在0.1-10米范围内，面积通常很大（>1000）
        if 0.1 < msg.z < 10.0:
            self.target_depth = msg.z
            self.has_depth_info = True
        else:
            # 备用：使用面积估算
            self.target_area = msg.z
            self.has_depth_info = False
        
        self.target_detected = True
        self.target_lost_time = self.get_clock().now().seconds_nanoseconds()[0]
    
    def odom_callback(self, msg):
        """接收里程计信息（预留用于更复杂的控制）"""
        pass
    
    def control_loop(self):
        """主控制循环 - 10Hz运行"""
        twist = Twist()
        
        # 检查目标是否丢失
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if not self.target_detected or (current_time - self.target_lost_time > self.target_timeout):
            # 目标丢失，停止运动
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # 重置PID积分项
            self.error_angle_integral = 0.0
            self.error_dist_integral = 0.0
            return
        
        # ========== 角度控制（转向）==========
        # 误差：目标x偏移（图像中心为0）
        error_angle = self.target_x
        
        # 计算PID输出
        self.error_angle_integral += error_angle * 0.1  # 积分
        self.error_angle_integral = self.clamp(self.error_angle_integral, -1000, 1000)  # 抗积分饱和
        
        error_angle_derivative = (error_angle - self.error_angle_prev) / 0.1  # 微分
        self.error_angle_prev = error_angle
        
        angular_z = -(self.kp_angle * error_angle + 
                      self.ki_angle * self.error_angle_integral + 
                      self.kd_angle * error_angle_derivative)
        
        # ========== 距离控制（前进/后退）==========
        if self.has_depth_info and self.min_valid_depth < self.target_depth < self.max_valid_depth:
            # 使用深度信息进行精确距离控制
            # 误差：当前深度与目标深度的差
            # 正误差表示目标太远，需要前进
            error_dist = self.target_depth - self.target_depth_setpoint
            
            # 计算PID输出
            self.error_dist_integral += error_dist * 0.1
            self.error_dist_integral = self.clamp(self.error_dist_integral, -5.0, 5.0)
            
            error_dist_derivative = (error_dist - self.error_dist_prev) / 0.1
            self.error_dist_prev = error_dist
            
            linear_x = self.kp_dist * error_dist + \
                       self.ki_dist * self.error_dist_integral + \
                       self.kd_dist * error_dist_derivative
            
            control_mode = "深度控制"
        else:
            # 回退到面积估算（深度不可用）
            # 误差：目标面积与设定值的差
            error_dist = 15.0 - self.target_area / 1000.0  # 缩放到合理范围
            
            self.error_dist_integral += error_dist * 0.1
            self.error_dist_integral = self.clamp(self.error_dist_integral, -50, 50)
            
            error_dist_derivative = (error_dist - self.error_dist_prev) / 0.1
            self.error_dist_prev = error_dist
            
            # 使用较小的增益
            linear_x = 0.05 * error_dist
            
            control_mode = "面积估算"
        
        # ========== 速度限制 ==========
        linear_x = self.clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        
        # 如果角度误差较大，优先转向，减少前进速度
        if abs(error_angle) > 100:
            linear_x *= 0.3  # 角度偏差大时减速
        
        # 发布速度指令
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        # 调试信息（每2秒输出一次）
        if int(current_time * 10) % 20 == 0:
            if self.has_depth_info:
                self.get_logger().info(
                    f'[{control_mode}] 深度: {self.target_depth:.2f}m, 偏移: {self.target_x:.0f}px | '
                    f'速度: 线={linear_x:.2f}, 角={angular_z:.2f}'
                )
            else:
                self.get_logger().info(
                    f'[{control_mode}] 面积: {self.target_area:.0f}, 偏移: {self.target_x:.0f}px | '
                    f'速度: 线={linear_x:.2f}, 角={angular_z:.2f}'
                )
    
    def clamp(self, value, min_val, max_val):
        """限制值在范围内"""
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止小车
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
