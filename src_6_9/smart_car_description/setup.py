from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'smart_car_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weiyu',
    maintainer_email='weiyu@todo.todo',
    description='基于ROS2和OpenCV的智能小车，具备实时图像识别与自主跟踪功能',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_processor_node = smart_car_description.vision_processor:main',
            'motion_controller_node = smart_car_description.motion_controller:main',
        ],
    },
)
