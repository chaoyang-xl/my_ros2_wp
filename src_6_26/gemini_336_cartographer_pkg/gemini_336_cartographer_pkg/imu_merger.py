import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuMerger(Node):
    def __init__(self):
        super().__init__('imu_merger_node')
        
        self.latest_accel = None
        
        # 订阅分开的加速度和陀螺仪话题
        self.accel_sub = self.create_subscription(Imu, '/camera/accel/sample', self.accel_callback, 10)
        self.gyro_sub = self.create_subscription(Imu, '/camera/gyro/sample', self.gyro_callback, 10)
            
        # 发布合并后的 IMU 话题
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', 10)
        self.get_logger().info("IMU 合并节点已启动！正在等待 accel 和 gyro 数据...11")

    def accel_callback(self, msg):
        # 保存最新的加速度数据
        self.latest_accel = msg

    def gyro_callback(self, msg):
        # 陀螺仪频率通常较高，以陀螺仪的回调为触发器发布合并数据
        if self.latest_accel is None:
            return
            
        merged_msg = Imu()
        # 1. 使用陀螺仪的时间戳和坐标系 (Cartographer 对时间戳极其敏感，必须统一)
        merged_msg.header = msg.header
        merged_msg.header.stamp = self.get_clock().now().to_msg()  # 使用当前时间戳，确保与 Cartographer 同步
        # 2. 缝合线加速度和角速度
        merged_msg.linear_acceleration = self.latest_accel.linear_acceleration
        merged_msg.angular_velocity = msg.angular_velocity
        
        # 3. 缝合协方差矩阵 (可选，但为了严谨带上)
        merged_msg.linear_acceleration_covariance = self.latest_accel.linear_acceleration_covariance
        merged_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        
        # 4. 姿态信息 (由于我们没有计算姿态，按 ROS 标准将协方差第一位设为 -1 表示无姿态)
        merged_msg.orientation.w = 1.0
        merged_msg.orientation_covariance[0] = -1.0
        
        # 5. 发布出去！
        self.imu_pub.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()