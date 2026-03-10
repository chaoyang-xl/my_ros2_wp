from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weiyu',
    maintainer_email='weiyu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main",
            "robot_news_station_node = my_py_pkg.robot_news_station:main",
            "smartphone_node = my_py_pkg.smartphone:main",
            "number_publisher_node = my_py_pkg.number_publisher:main",
            "number_counter_node = my_py_pkg.number_counter:main",
            "add_two_ints_server_node = my_py_pkg.add_two_ints_server:main",
            "client_with_no_oop_node = my_py_pkg.client_with_no_oop:main",
            "add_two_ints_client_node = my_py_pkg.add_two_ints_client:main",
            "hardware_status_publisher_node = my_py_pkg.hardware_status_publisher:main",
            "get_area_service_node = my_py_pkg.get_area:main",
            "led_panel_node = my_py_pkg.led_panel:main",
            "battery_node = my_py_pkg.battery:main",
        ],
    },
)
