from setuptools import setup
import os
from glob import glob

package_name = 'rear_camera_pose'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@example.com',
    description='Rear camera YOLO-pose ROS2 node with posture classification',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rear_pose_node = rear_camera_pose.rear_pose_node:main',
            'usb_cam_publisher = rear_camera_pose.usb_cam_publisher:main',
        ],
    },
)
