from setuptools import setup
from glob import glob
import os

package_name = 'opi_yolo_rknn_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@example.com',
    description='YOLO and YOLO-pose recorder for Orange Pi 5 Plus RK3588',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ai_recorder_node = opi_yolo_rknn_recorder.ai_recorder_node:main',
            'check_topics = opi_yolo_rknn_recorder.check_topics:main',
        ],
    },
)
