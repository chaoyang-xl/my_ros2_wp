"""Launch rear camera pose detection — camera publisher + YOLO pose node.

Architecture:
  usb_cam_publisher (V4L2 → /rear_camera/image_raw)  ← optional
  rear_pose_node   (subscribes, runs YOLO-pose inference, HTTP preview)

Examples:
  # Default: start both (USB camera + pose)
  ros2 launch rear_camera_pose rear_pose.launch.py

  # Different camera device
  ros2 launch rear_camera_pose rear_pose.launch.py camera_device:=/dev/video0

  # External camera driver (no built-in publisher)
  ros2 launch rear_camera_pose rear_pose.launch.py camera_device:=''

  # Lower confidence, higher resolution
  ros2 launch rear_camera_pose rear_pose.launch.py conf:=0.35 width:=1280 height:=720
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Camera publisher ----
    camera_device_arg = DeclareLaunchArgument(
        "camera_device", default_value="/dev/video2",
        description="V4L2 device path",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/rear_camera/image_raw",
        description="Image topic (bridge between publisher and pose node)",
    )
    width_arg = DeclareLaunchArgument(
        "width", default_value="640", description="V4L2 capture width"
    )
    height_arg = DeclareLaunchArgument(
        "height", default_value="480", description="V4L2 capture height"
    )
    fps_arg = DeclareLaunchArgument(
        "fps", default_value="15", description="V4L2 capture FPS"
    )

    # ---- Model ----
    pose_model_arg = DeclareLaunchArgument(
        "pose_model", default_value="/home/weiyu/vscode_workspace/models/yolo11n-pose.pt",
    )
    imgsz_arg = DeclareLaunchArgument("imgsz", default_value="640")
    conf_arg = DeclareLaunchArgument("conf", default_value="0.25")
    iou_arg = DeclareLaunchArgument("iou", default_value="0.45")
    frame_skip_arg = DeclareLaunchArgument("frame_skip", default_value="0")

    # ---- Output ----
    output_dir_arg = DeclareLaunchArgument("output_dir", default_value="~/rear_pose_results")
    result_topic_arg = DeclareLaunchArgument("result_topic", default_value="/rear_pose/results_json")
    publish_debug_image_arg = DeclareLaunchArgument("publish_debug_image", default_value="true")
    debug_image_topic_arg = DeclareLaunchArgument("debug_image_topic", default_value="/rear_pose/debug_image")
    save_every_n_arg = DeclareLaunchArgument("save_every_n", default_value="30")

    # ---- HTTP ----
    enable_http_arg = DeclareLaunchArgument("enable_http", default_value="true")
    http_port_arg = DeclareLaunchArgument("http_port", default_value="8089")
    mjpeg_fps_arg = DeclareLaunchArgument("mjpeg_fps", default_value="8")

    # ---- Nodes ----

    usb_cam_node = Node(
        package="rear_camera_pose",
        executable="usb_cam_publisher",
        name="usb_cam_publisher",
        output="screen",
        parameters=[{
            "camera_device": LaunchConfiguration("camera_device"),
            "image_topic": LaunchConfiguration("image_topic"),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
        }],
    )

    rear_pose_node = Node(
        package="rear_camera_pose",
        executable="rear_pose_node",
        name="rear_pose_node",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "pose_model": LaunchConfiguration("pose_model"),
            "imgsz": LaunchConfiguration("imgsz"),
            "conf": LaunchConfiguration("conf"),
            "iou": LaunchConfiguration("iou"),
            "frame_skip": LaunchConfiguration("frame_skip"),
            "output_dir": LaunchConfiguration("output_dir"),
            "result_topic": LaunchConfiguration("result_topic"),
            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            "debug_image_topic": LaunchConfiguration("debug_image_topic"),
            "save_every_n": LaunchConfiguration("save_every_n"),
            "enable_http": LaunchConfiguration("enable_http"),
            "http_host": "0.0.0.0",
            "http_port": LaunchConfiguration("http_port"),
            "mjpeg_fps": LaunchConfiguration("mjpeg_fps"),
        }],
    )

    return LaunchDescription([
        camera_device_arg, image_topic_arg, width_arg, height_arg, fps_arg,
        pose_model_arg, imgsz_arg, conf_arg, iou_arg, frame_skip_arg,
        output_dir_arg, result_topic_arg, publish_debug_image_arg,
        debug_image_topic_arg, save_every_n_arg,
        enable_http_arg, http_port_arg, mjpeg_fps_arg,
        usb_cam_node,
        rear_pose_node,
    ])
