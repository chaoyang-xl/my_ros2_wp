import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Pacemaker(Node):
    def __init__(self):
        super().__init__('system_pacemaker')
        # 设置强力的 Best Effort QoS，充当万能诱饵
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 1. 强行订阅点云，保持相机苏醒
        self.sub_points = self.create_subscription(PointCloud2, '/camera/depth/points', self.dummy_callback, qos)
        # 2. 强行订阅雷达，保持转换节点苏醒
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.dummy_callback, qos)
        
        self.get_logger().info("相机和雷达已被强制唤醒，绝不休眠。")

    def dummy_callback(self, msg):
        # 只需要订阅，不需要处理数据，就像个安静的黑洞
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Pacemaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()