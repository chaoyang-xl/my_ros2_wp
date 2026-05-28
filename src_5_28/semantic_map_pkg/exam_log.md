测试日志
5.15
cd /home/weiyu/vscode_workspace/ros2_wp
colcon build --packages-select semantic_map_pkg
source install/setup.bash
ros2 run semantic_map_pkg semantic_map_marker_demo
新增semantic_map_marker_demo，发布marker测试数据，在地图上显示文字和框
