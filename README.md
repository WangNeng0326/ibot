# ibot
Bachelor graduation project:Design and Implementation of Mecanum Wheeled Autonomous Navigation Robot Based on ROS

1、SSH连接小车（个人电脑连接上小车的wifi）  
ssh sz@192.168.5.101  
然后输入密码123456  
2、打开ROS  
roscore  
3、打开键盘控制命令  
roslaunch ibot_teleop ibot_cmd.launch  
roslaunch ibot_teleop ibot_keyboard.launch  
4、打开pc到stm32的通信节点  
rosrun ibot_driver ibot_pc2stm32  
rosrun ibot_driver ibot_bringup  
rosrun ibot_driver ibot_bringup_slam  (不发布里程计信息)  
5、打开激光雷达  
roslaunch delta_lidar delta_driver.launch  (minipc)  
6、slam建图  
	hector_slam:(不需要里程计)  
		roslaunch ibot_run ibot_hector.launch         (car)  
		roslaunch ibot_teleop ibot_keyboard.launch  
		roslaunch ibot_navigation mapping_rviz.launch (pc)  
	gmapping:(需要里程计)  
		roslaunch ibot_run ibot_gmapping.launch       (car)  
		roslaunch ibot_navigation gmapping.launch  
		roslaunch ibot_teleop ibot_keyboard.launch  
		roslaunch ibot_navigation mapping_rviz.launch (pc)  
	cartographer:(需要里程计)  
		roscore  
		roslaunch delta_lidar delta_driver  
		roslaunch ibot_driver ibot_bringup  
		roslaunch ibot_description display_ibot.launch   
		roslaunch cartographer_ros cartographer_demo_ibot.launch  
		roslaunch ibot_teleop ibot_keyboard.launch  
		roslaunch cartographer_ros cartographer_rviz.launch  (pc)  
保存地图：  
rosrun map_server map_saver -f ($find ibot_navigation)/maps/mymap  
rosrun map_server map_saver -f  ~/ros_ibot/catkin_ws/src/ibot_navigation/maps/hector_slam  
7、move_base导航  
roslaunch delta_lidar delta_driver.launch  
roslaunch ibot_driver ibot_bringup  
roslaunch ibot_navigation ibot_move.launch  
roslaunch ibot_navigation rviz.launch  
或者：  
	roslaunch ibot_run ibot_nav.launch  
	roslaunch ibot_navigation rviz.launch  
