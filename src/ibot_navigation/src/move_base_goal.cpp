#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_goal");
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); //初始化actionlib的客户端
    ROS_INFO("Waitting for move base server...");

	if(!ac.waitForServer(ros::Duration(60))) //判断是否链接到move base服务器
	{
		ROS_ERROR("Cant connected to move base server.");
		return -1;
	}
	else
	{
		ROS_INFO("Already connected to move base server");
		ROS_INFO("Starting navigation test");
	}
    move_base_msgs::MoveBaseGoal goal;   //初始化目标点
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //判断到达目标点后的状态
	{
		ROS_INFO("Hooray, the base moved 1 meter forward");
	}
	else
	{
		ROS_INFO("The base failed to move forward 1 meter for some reason");
			//ac.cancelGoal(); //取消该点导航
	}
    return 0;
}