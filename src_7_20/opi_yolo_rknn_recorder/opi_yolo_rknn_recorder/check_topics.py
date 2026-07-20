#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class TopicChecker(Node):
    def __init__(self):
        super().__init__('check_topics')
        self.timer = self.create_timer(1.0, self.print_topics)
        self.done = False

    def print_topics(self):
        if self.done:
            return
        topics = self.get_topic_names_and_types()
        for name, types in sorted(topics):
            if 'camera' in name or 'image' in name or 'yolo' in name:
                self.get_logger().info(f'{name}: {types}')
        self.done = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TopicChecker()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
