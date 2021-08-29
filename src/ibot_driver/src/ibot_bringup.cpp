/**
**  * @function 读取ibot 底盘上STM32发送过来的数据帧 并发布里程计数据
                        同时 接受cmd_vel 话题的数据，将其转化成转速指令 然后下发到底盘的STM32控制器中
**/


#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>

#include <serial/serial.h>
#include <std_msgs/String.h>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace boost::asio;     ////定义一个命名空间，用于后面的读写操作

float WHEEL_RATIO =30.0; 		//减速比
float WHEEL_K=0.29;                              // abs(X) + abs(Y)
float WHEEL_D=0.097; 			//轮子直径
float WHEEL_R=WHEEL_D/2.0; 			//轮子直径
float WHEEL_PI=3.141693; 			//pi

struct timeval time_val; //time varible
struct timezone tz;
double time_stamp;
serial::Serial ros_ser;
ros::Publisher odom_pub;

typedef struct{
		int16_t	 	speed_rpm;       //转速
		uint16_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
}moto_measure_t;
typedef struct{
		uint16_t 	ax,ay,az;		 
 		uint16_t 	gx,gy,gz;	
 		uint16_t 	mx,my,mz;	
		float pitch,roll,yaw;
		float pitch_rad,roll_rad,yaw_rad;
}imu_measure_t;

moto_measure_t moto_chassis[4] = {0};
imu_measure_t imu_chassis;  //IMU 数据
uint16_t Ultrasonic [10];   //超声波数据

union floatData //union的作用为实现char数组和float之间的转换
{
    int32_t int32_dat;
    unsigned char byte_data[4];
}motor_upload_counter,total_angle,round_cnt;

typedef union{
	unsigned char cvalue[4];
	float fvalue;
}float_union; 

float_union MotorCount1_Autual,MotorCount2_Autual,MotorCount3_Autual,MotorCount4_Autual;//从stm32传来的电机转速统计

union IntData //union的作用为实现char数组和int16数据类型之间的转换
{
    int16_t int16_dat;
    unsigned char byte_data[2];
}speed_rpm;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
void send_speed_to_chassis(float x,float y,float w);
void send_rpm_to_chassis( float w1, float w2, float w3, float w4);
//void clear_odometry_chassis(void);
bool analy_uart_recive_data( std_msgs::String serial_data);
void calculate_position_for_odometry(void);
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w);

ros::Time current_time, last_time;//定义里程计计算时需要的时间
int main(int argc, char *argv[])
{
    string out_result;
    bool uart_recive_flag;

    ros::init(argc, argv, "ibot_bringup");
    ros::NodeHandle n;
	 //订阅主题command
	ros::Subscriber command_sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);

    current_time=ros::Time::now();
    last_time=ros::Time::now();
	 //发布主题sensor
	   // ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
     odom_pub= n.advertise<nav_msgs::Odometry>("/odom", 20); //定义要发布/odom主题
	// 开启串口模块
    try
    {
        ros_ser.setPort("/dev/stm32f4");
        ros_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
		ros_ser.flushInput(); //清空缓冲区数据
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ros_ser.isOpen()){
        ros_ser.flushInput(); //清空缓冲区数据
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }	
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    while (ros::ok())
    {
        if(ros_ser.available()){ 
              ROS_INFO_STREAM("Reading from serial port\n"); 
              std_msgs::String serial_data; 
              serial_data.data = ros_ser.read(ros_ser.available()); 
              uart_recive_flag=analy_uart_recive_data(serial_data);
              if(uart_recive_flag)
              {
                  calculate_position_for_odometry();
              }
              else
              {
                  ros_ser.flushInput(); //清空缓冲区数据
              }
             // ROS_INFO_STREAM("Read: " << serial_data.data); 
          }
          //处理ROS的信息，比如订阅消息,并调用回调函数 
          ros::spinOnce(); 
          loop_rate.sleep();   
    }
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();
    return 1;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{

  float speed_x,speed_y,speed_w;
  float v1=0,v2=0,v3=0,v4=0;
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  speed_w = msg->angular.z;
  
  v1 =speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
  v2 =speed_x+speed_y-WHEEL_K*speed_w;
  v3 =speed_x-speed_y+WHEEL_K*speed_w;
  v4 =speed_x+speed_y+WHEEL_K*speed_w;

  v1 =v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度　RPM
  v2 =v2/(2.0*WHEEL_R*WHEEL_PI);
  v3 =v3/(2.0*WHEEL_R*WHEEL_PI);
  v4 =v4/(2.0*WHEEL_R*WHEEL_PI);
  
  v1 =v1*WHEEL_RATIO*60;    //转换为电机速度　单位　ＲＰＭ
  v2 =v2*WHEEL_RATIO*60;
  v3 =v3*WHEEL_RATIO*60;
  v4 =v4*WHEEL_RATIO*60;
  
  
  send_rpm_to_chassis(v1,v2,v3,v4);	 
 //send_rpm_to_chassis(200,200,200,200);	
   //ROS_INFO_STREAM("v1: "<<v1<<"      v2: "<<v2<<"      v3: "<<v3<<"      v4: "<<v4);
  //ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z);
}
/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　ｒｐｍ
 */
void send_rpm_to_chassis( float w1, float w2, float w3, float w4)
{
 
  uint8_t data_tem[50];
  uint8_t check=0;
  unsigned int speed_0ffset=500; //转速偏移
  float_union V_rpm1,V_rpm2,V_rpm3,V_rpm4;
  V_rpm1.fvalue=w1/speed_0ffset;
  V_rpm2.fvalue=w2/speed_0ffset;
  V_rpm3.fvalue=w3/speed_0ffset;
  V_rpm4.fvalue=w4/speed_0ffset;

  memset(data_tem,0,sizeof(data_tem));
  data_tem[0] =0xAA;
  data_tem[1] =0xBB;
 
  data_tem[2] =V_rpm1.cvalue[0]; // 
  data_tem[3] =V_rpm1.cvalue[1];
  data_tem[4] =V_rpm1.cvalue[2];// 
  data_tem[5] =V_rpm1.cvalue[3];
  
  data_tem[6] =V_rpm2.cvalue[0]; // 
  data_tem[7] =V_rpm2.cvalue[1];
  data_tem[8] =V_rpm2.cvalue[2];// 
  data_tem[9] =V_rpm2.cvalue[3];
  
  data_tem[10] =V_rpm3.cvalue[0]; // 
  data_tem[11] =V_rpm3.cvalue[1];
  data_tem[12] =V_rpm3.cvalue[2];// 
  data_tem[13] =V_rpm3.cvalue[3];

  data_tem[14] =V_rpm4.cvalue[0]; // 
  data_tem[15] =V_rpm4.cvalue[1];
  data_tem[16] =V_rpm4.cvalue[2];// 
  data_tem[17] =V_rpm4.cvalue[3];

 for(unsigned char i=0;i<18;i++)
  {
    check+=data_tem[i];
  }
  data_tem[18]=check;
  data_tem[19] =0xFF;
 
 ros_ser.write(data_tem,20);
}

//分析串口接受的数据,找到帧头
bool analy_uart_recive_data( std_msgs::String serial_data)
{
    uint16_t len=0,i=0,j=0;
    unsigned char reviced_tem[500];
    uint8_t check=0;//校验数据
    uint16_t Buffer_Szie=20;//接受数据的长度
    unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
    uint16_t header_count=0,step=0;//计数这个数据序列中有多少个帧头
    len=serial_data.data.size();
    if(len<1||len>500)
    {
    	ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
        return false; //数据长度太短　      
    }
    ROS_INFO_STREAM("Read: " << serial_data.data.size() );
    for(i=0;i<len;i++)
    {
        tem_last=tem_curr;
        tem_curr=serial_data.data.at(i);
        if(tem_last==0XCC&&tem_curr==0XDD&&rec_flag==0)//在接受数据中找到帧头
        {
            rec_flag=1;
            reviced_tem[j++]=tem_last;
            reviced_tem[j++]=tem_curr;
            ROS_INFO_STREAM("Found frame head");
        }
        else if(rec_flag==1)
        {
            reviced_tem[j++]=serial_data.data.at(i);
            if(tem_curr==0XEE)
            {
                header_count++;
                rec_flag=2;
            }
        }
        else rec_flag=0;
    }
     step=0;
    // for(i=0;i<header_count;i++)
    // {
        if(reviced_tem[0]==0XCC&&reviced_tem[1]==0XDD&&reviced_tem[Buffer_Szie-1]==0XEE)
        {//检查帧头帧尾是否完整
            for(unsigned char m=0;m<18;m++)
            {
                check+=reviced_tem[m];
            }
            if(check==reviced_tem[18])
            {
                MotorCount1_Autual.cvalue[0]=reviced_tem[2];
                MotorCount1_Autual.cvalue[1]=reviced_tem[3];
                MotorCount1_Autual.cvalue[2]=reviced_tem[4];
                MotorCount1_Autual.cvalue[3]=reviced_tem[5];

                MotorCount2_Autual.cvalue[0]=reviced_tem[6];
                MotorCount2_Autual.cvalue[1]=reviced_tem[7];
                MotorCount2_Autual.cvalue[2]=reviced_tem[8];
                MotorCount2_Autual.cvalue[3]=reviced_tem[9];

                MotorCount3_Autual.cvalue[0]=reviced_tem[10];
                MotorCount3_Autual.cvalue[1]=reviced_tem[11];
                MotorCount3_Autual.cvalue[2]=reviced_tem[12];
                MotorCount3_Autual.cvalue[3]=reviced_tem[13];

                MotorCount4_Autual.cvalue[0]=reviced_tem[14];
                MotorCount4_Autual.cvalue[1]=reviced_tem[15];
                MotorCount4_Autual.cvalue[2]=reviced_tem[16];
                MotorCount4_Autual.cvalue[3]=reviced_tem[17];
            }
            cout<<"V1:"<<MotorCount1_Autual.fvalue<<endl;
            cout<<"V2:"<<MotorCount2_Autual.fvalue<<endl;
            cout<<"V3:"<<MotorCount3_Autual.fvalue<<endl;
            cout<<"V4:"<<MotorCount4_Autual.fvalue<<endl;
        }
        else
        {
 		    ROS_WARN_STREAM("frame head is wrong" ); 
             return  false;	           
        }
         return  true;	      
    // }
}

/*
**利用里程计数据实现位置估计
*/
float position_x=0,position_y=0,position_w=0;//定义X,Y,W三个方向的位移
float Vx,Vy,Vth;
void calculate_position_for_odometry(void)
{
    float Speed_Motor1_Actual,Speed_Motor2_Actual,Speed_Motor3_Actual,Speed_Motor4_Actual;//电机的实际速度
    float dt;//定义时间
    float delta_x,delta_y,delta_th;//定义X,Y,W放个方向的位移增量
    current_time = ros::Time::now();//获取当前时间
    dt = (current_time-last_time).toSec();

    Speed_Motor1_Actual=(MotorCount1_Autual.fvalue)*0.09234;//经过换算可以直接由转速得到速度
    Speed_Motor2_Actual=(MotorCount2_Autual.fvalue)*0.09234;
    Speed_Motor3_Actual=(MotorCount3_Autual.fvalue)*0.09234;
    Speed_Motor4_Actual=(MotorCount4_Autual.fvalue)*0.09234;
    //系数是由麦克拉姆轮运动学正解算得到的F矩阵
    Vx=0.25*Speed_Motor1_Actual+0.25*Speed_Motor2_Actual+0.25*Speed_Motor3_Actual+0.25*Speed_Motor4_Actual;
    Vy=-0.25*Speed_Motor1_Actual+0.25*Speed_Motor2_Actual-0.25*Speed_Motor3_Actual+0.25*Speed_Motor4_Actual;
    Vth=-0.8621*Speed_Motor1_Actual-0.8621*Speed_Motor2_Actual+0.8621*Speed_Motor3_Actual+0.8621*Speed_Motor4_Actual;

    delta_x=(Vx*cos(position_w)-Vy*sin(position_w))*dt;
    delta_y=(Vx*sin(position_w)+Vy*cos(position_w))*dt;
    delta_th=Vth*dt;
    //计算X Y W三个方向的位移增量
    position_x += delta_x;
    position_y += delta_y;
    position_w += delta_th;

    cout<<"position_x:  "<<position_x<<"   position_y: " <<position_y<<"   position_w: " <<position_w*57.3<<endl;
    cout<<"linear_x:  "<<Vx<<"   linear_y: " <<Vy<<"   linear_w: " <<Vth<<endl;

    //构造里程计的信息发布
    publish_odomtery(position_x,position_y,position_w,Vx,Vy,Vth);
    last_time = current_time;
}

void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
	geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
	geometry_msgs::Quaternion odom_quat;   //四元数变量
	nav_msgs::Odometry odom;  //定义里程计对象

    //里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf::createQuaternionMsgFromYaw(oriention);//将偏航角转换成四元数   
    //载入坐标(tf)变换时间戳
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";     
    odom_trans.child_frame_id = "base_link";  
    //tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = position_x;
    odom_trans.transform.translation.y = position_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;       
    //发布tf坐标变化
    odom_broadcaster.sendTransform(odom_trans);

    //载入里程计时间戳
    odom.header.stamp = ros::Time::now(); 
    //里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";       
    //里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = position_x;     
    odom.pose.pose.position.y = position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;       
    //载入线速度和角速度
    odom.twist.twist.linear.x = vel_linear_x;
    odom.twist.twist.linear.y = vel_linear_y;
    odom.twist.twist.angular.z = vel_linear_w;    
    //发布里程计
    odom_pub.publish(odom);    
}