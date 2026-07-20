"""Optional Cartographer SLAM launch for users who provide a Lua config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Cartographer and its occupancy-grid publisher."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    configuration_directory = LaunchConfiguration("configuration_directory")
    configuration_basename = LaunchConfiguration("configuration_basename")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
        DeclareLaunchArgument(
            "configuration_directory",
            description="Directory containing the Cartographer Lua configuration.",
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            description="Cartographer Lua configuration filename.",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", configuration_directory,
                "-configuration_basename", configuration_basename,
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-resolution", LaunchConfiguration("resolution"),
                "-publish_period_sec", LaunchConfiguration("publish_period_sec"),
            ],
        ),
    ])
