from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('output_dir', default_value='/home/weiyu/vscode_workspace/yolo_results_snapshot'),
        DeclareLaunchArgument('detect_model', default_value='/home/weiyu/vscode_workspace/models/yolo11n.pt'),
        DeclareLaunchArgument('pose_model', default_value='/home/weiyu/vscode_workspace/models/yolo11n-pose.pt'),
        DeclareLaunchArgument('enable_detect', default_value='true'),
        DeclareLaunchArgument('enable_pose', default_value='true'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument('iou', default_value='0.45'),
        DeclareLaunchArgument('frame_skip', default_value='0'),
        DeclareLaunchArgument('save_every_n', default_value='30'),
        DeclareLaunchArgument('enable_http', default_value='true'),
        DeclareLaunchArgument('http_port', default_value='8088'),
    ]

    node = Node(
        package='opi_yolo_rknn_recorder',
        executable='ai_recorder_node',
        name='ai_recorder_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'detect_model': LaunchConfiguration('detect_model'),
            'pose_model': LaunchConfiguration('pose_model'),
            'enable_detect': LaunchConfiguration('enable_detect'),
            'enable_pose': LaunchConfiguration('enable_pose'),
            'imgsz': LaunchConfiguration('imgsz'),
            'conf': LaunchConfiguration('conf'),
            'iou': LaunchConfiguration('iou'),
            'frame_skip': LaunchConfiguration('frame_skip'),
            'save_every_n': LaunchConfiguration('save_every_n'),
            'enable_http': LaunchConfiguration('enable_http'),
            'http_port': LaunchConfiguration('http_port'),
        }]
    )
    return LaunchDescription(args + [node])
