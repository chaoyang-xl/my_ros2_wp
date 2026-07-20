"""Launch the Hybrid A* planner, controller and optional RViz UI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the planner launch description."""
    package_share = get_package_share_directory("hybrid_algorithm_pkg")
    default_params = os.path.join(package_share, "config", "planner_params.yaml")
    default_rviz = os.path.join(package_share, "config", "hybrid_algorithm.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    enable_controller = LaunchConfiguration("enable_controller")
    enable_rviz = LaunchConfiguration("enable_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("enable_controller", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        Node(
            package="hybrid_algorithm_pkg",
            executable="hybrid_astar_planner",
            name="hybrid_algorithm_planner",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="hybrid_algorithm_pkg",
            executable="pure_pursuit_controller",
            name="pure_pursuit_controller",
            output="screen",
            condition=IfCondition(enable_controller),
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(enable_rviz),
            arguments=["-d", default_rviz],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
