import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloPoseNode(Node):
    def __init__(self):
        super().__init__("yolo_pose_node")

        self.declare_parameter("model", "yolo11n-pose.pt")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("conf", 0.4)

        model_path = self.get_parameter("model").value
        image_topic = self.get_parameter("image_topic").value
        self.conf = float(self.get_parameter("conf").value)

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/yolo_pose/keypoints",
            10
        )

        self.get_logger().info(f"YOLO-pose node started")
        self.get_logger().info(f"Subscribing image topic: {image_topic}")
        self.get_logger().info(f"Model: {model_path}")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(frame, conf=self.conf, verbose=False)

        output = Float32MultiArray()
        data = []

        for result in results:
            if result.boxes is None or result.keypoints is None:
                continue

            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            keypoints = result.keypoints.data.cpu().numpy()

            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes[i]
                score = scores[i]

                data.extend([
                    float(i),
                    float(x1), float(y1), float(x2), float(y2),
                    float(score)
                ])

                # keypoints shape: [17, 3], each keypoint = x, y, confidence
                data.extend(keypoints[i].reshape(-1).astype(float).tolist())

        output.data = data
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
