1  	保存第一步跑完的地图 安装过ros-jazzy-navigation2这个工具
	输入下面这行命令，地图就会保存到你的主目录 (~) 下：
	ros2 run nav2_map_server map_saver_cli -f ~/my_first_map
	my_first_map.pgm：这是一张图片，你可以双击打开。
	白色：表示空旷区域（机器人走过的地方）。
	黑色：表示障碍物（墙、箱子）。
	灰色：表示未知区域（雷达没扫到的地方）。
	my_first_map.yaml：这是一个文本文件，记录了地图的分辨率、原点坐标等信息，导航时需要读取这个文件。


2  	使用NAV2导航时先打开gazebo环境获取数据再打开rviz进行acml定位和导航
	ros2 launch fishbot_description display_rviz2.launch.py 
	ros2 launch fishbot_description nav2.launch.py 
