/*
   function:minipc對stm32串口发送数据
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>

serial::Serial ser;//创建一个ser类

#define	sendchar 15
unsigned char s_buffer[sendchar];//发送缓存
typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union; 

void data_pack(const geometry_msgs::Twist& cmd_vel)
{
 float_union Vx,Vy,Ang_v;
	Vx.fvalue = cmd_vel.linear.x;
	Vy.fvalue = cmd_vel.linear.y;
	Ang_v.fvalue = cmd_vel.angular.z;
	
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cvalue[0];
	s_buffer[3] = Vx.cvalue[1];
	s_buffer[4] = Vx.cvalue[2];
	s_buffer[5] = Vx.cvalue[3];
	//Vy
	s_buffer[6] = Vy.cvalue[0];
	s_buffer[7] = Vy.cvalue[1];
	s_buffer[8] = Vy.cvalue[2];
	s_buffer[9] = Vy.cvalue[3];
	//Ang_v
	s_buffer[10] = Ang_v.cvalue[0];
	s_buffer[11] = Ang_v.cvalue[1];
	s_buffer[12] = Ang_v.cvalue[2];
	s_buffer[13] = Ang_v.cvalue[3];
	//CRC
	s_buffer[14] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7]^
					s_buffer[8]^s_buffer[9]^s_buffer[10]^s_buffer[11]^s_buffer[12]^s_buffer[13];
	

	ser.write(s_buffer,sendchar);
}

void TwistCallback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("I heard linear velocity: x-[%f]",cmd_vel.linear.x);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
    data_pack(cmd_vel);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ibot_pc2stm32");
    ros::NodeHandle n;
    ros::Subscriber write_sub = n.subscribe("/cmd_vel",1,TwistCallback);

    try
    {
        ser.setPort("/dev/stm32f4");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

   ros::spin();

    return 0;
}
