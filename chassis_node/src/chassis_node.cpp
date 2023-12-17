#include <sstream>
#include <vector>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <pthread.h>
#include <mutex>
#include <unistd.h>
#include <thread>
#include "../include/base_serial.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

using namespace std;

//使用robot_pose_ekf所需的odom协方差

// 姿势协方差
boost::array<double, 36> odom_pose_covariance = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

// 转动协方差
boost::array<double, 36> odom_twist_covariance = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};


BaseSerial* BSerial;
//小车轮距,单位：m
double RACEBOT_WIDTH=0.29; 
//小车轮半径,单位：m
double RACEBOT_WHEEL_R=0.0775;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	//ROS_INFO("recieved: linear: x=[%f] y=[%f] z=[%f];angular: x=[%f] y=[%f] z=[%f]", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
	//由线速度和角速度计算左右电机转速，单位rpm

	//小车中心前进速度，单位: m/s
	double vel_cent=msg->linear.x*5.5; //嫌太慢,*2.50
	//小车z轴上的角速度，单位: rad/s
	double vel_th=-msg->angular.z*3.0;
	//左右轮速度
	double vel_l=vel_cent-vel_th*RACEBOT_WIDTH/2.0;
	double vel_r=vel_cent+vel_th*RACEBOT_WIDTH/2.0;
	//以下代码将原地转向改为车宽半径转向
	//if(vel_th>0){
	//	vel_l=vel_cent+vel_th*RACEBOT_WIDTH;
	//	vel_r=vel_cent;
	//}
	//if(vel_th<0){
	//	vel_l=vel_cent;
        //        vel_r=vel_cent+abs(vel_th)*RACEBOT_WIDTH;
	//}else{
	//	vel_l=vel_cent;
	//	vel_r=vel_cent;
	//}

	//左右轮转速
	short n_left=(short)(60.0*vel_l/(2.0*3.1415926*0.0775));
	short n_right=(short)(60.0*vel_r/(2.0*3.1415926*0.0775));
	ROS_INFO("----------------left n : %d",n_left);
	ROS_INFO("----------------left n : %d",n_right);

	// ROS_INFO("-------------------------------------");
	// ROS_INFO("vel_cent: %f\t vel_th: %f\t", vel_cent, vel_th);
	// ROS_INFO("vel_l: %f\t vel_r: %f\t", vel_l, vel_r);
	// ROS_INFO("n_l: %d\t n_r: %d\t", n_left, n_right);

	BSerial->InputMotSpeed(n_left,n_right);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_node");
	ros::NodeHandle nh;
	//发布小车状态信息
	ros::Publisher  pub = nh.advertise<std_msgs::String>("comstatus", 20);
	//发布odom主题消息
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	//广播维护里程计tf坐标
	tf::TransformBroadcaster odom_broadcaster;
	//订阅速度指令
	ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 20, twistCallback);
	//创建时间戳变量
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	//里程变量
	//x中心位移,绕z轴角度值
	double x=0.0;
	double y=0.0;
	double th=0.0;
	//左右轮编码器数
	int enc_leftWheel=0; 
	int enc_rightWheel=0;
	//初始化接收串口数据结构体 
	MotoRev motoRevData;
	motoRevData.leftFrontMoto=0;
	motoRevData.rightFrontMoto=0;
	motoRevData.leftBehindMoto=0;
	motoRevData.rightBehindMoto=0;
	motoRevData.batteryLevel=0;
	motoRevData.batteryVoltage=0;

	//创建串口对象
	BaseSerial *cs = new BaseSerial();
	BSerial = cs;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{  
		pthread_mutex_lock(&cs->msgMute);
		//取底盘数据
		if(!cs->buffer_rev.empty())
		{
		    // motoRevData = buf.mtext;
		    motoRevData = cs->buffer_rev[0];
		}

			//发布小车状态消息字符串
			string info("");
			std_msgs::String msg;
			string str_encLF=to_string(motoRevData.leftFrontMoto);
			string str_encRF=to_string(motoRevData.rightFrontMoto);
			string str_encLB=to_string(motoRevData.leftBehindMoto);
			string str_encRB=to_string(motoRevData.rightBehindMoto);
			string str_batteryLevel=to_string(motoRevData.batteryLevel);
			string str_batteryVoltage=to_string(motoRevData.batteryVoltage);
			info="leftF=";
			info.append(str_encLF);
			info.append(";");
			info.append("rightF=");
			info.append(str_encRF);
			info.append(";");
			info.append("leftB=");
			info.append(str_encLB);
			info.append(";");
			info.append("rightB=");
			info.append(str_encRB);
			info.append("batteryLevel=");
			info.append(str_batteryLevel);
			info.append("batteryVoltage=");
			info.append(str_batteryVoltage);
			msg.data = info;
			pub.publish(msg);
			//计算和发布odom
			//接收编码器数值(取前后轮平均值)
			int now_enc_leftWheel=(motoRevData.leftFrontMoto+motoRevData.leftBehindMoto)/2; 
			int now_enc_rightWheel=(motoRevData.rightFrontMoto+motoRevData.rightBehindMoto)/2; 
			//ROS_INFO("enc_leftWheel: %d\t enc_leftWheel: %d\t", now_enc_leftWheel, now_enc_rightWheel);
			//ROS_INFO("now_enc_leftWheel: %d\t now_enc_rightWheel: %d\t", enc_leftWheel, enc_rightWheel);
			//得到左右轮的位移
			double s_L=(now_enc_leftWheel-enc_leftWheel)*3.1415926*RACEBOT_WHEEL_R*2.0/11.0/90.0; // 电机减速比90:1,电机一圈11个脉冲
			double s_R=(now_enc_rightWheel-enc_rightWheel)*3.1415926*RACEBOT_WHEEL_R*2.0/11.0/90.0; // 电机减速比90:1,电机一圈11个脉冲
			//计算时间
			//记录目前时间
			current_time = ros::Time::now();
			double dt = (current_time - last_time).toSec();
			double x_cent = (s_L+s_R)/2.0/200; // 中心位移
			double th_z = -(s_R-s_L)/RACEBOT_WIDTH*10.0; // 旋转角
			double vx_cent = x_cent*dt; // 中心速度
			double vth_z = th_z*dt; // 旋转角速度
			//ROS_INFO("x_cent: %f\t th_z: %f\t dt:%f\t", x_cent, th_z, dt);
			//ROS_INFO("vx_cent: %f\t vth_z: %f\t", vx_cent, vth_z);
			
			if(vx_cent!=0){
				//计算x,y坐标
				double dx = vx_cent*cos(th_z);
				double dy = -vx_cent*sin(th_z);
				//累加x，y位移
				x += (cos(th)*dx-sin(th)*dy);
				y += (sin(th)*dx+cos(th)*dy);
				ROS_INFO("x: %f\t y: %f\t th: %f\t", x, y, th);
			}

			if(vth_z!=0){
				//累加角度
				th += th_z;
			}
			//创建tf消息
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//广播tf变换维护坐标（使用robot_pose_ekf的状态下应该注释掉）
			//odom_broadcaster.sendTransform(odom_trans);

			//发布odom主题消息
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//位置信息
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance = odom_pose_covariance;

			//速度信息
			odom.child_frame_id = "base_footprint";
			odom.twist.twist.linear.x = vx_cent;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.angular.z = vth_z;
			odom.twist.covariance = odom_twist_covariance;

			//发布odom主题
			odom_pub.publish(odom);
			//刷新时间戳
			last_time = current_time;
			//刷新编码器计数
			enc_leftWheel=now_enc_leftWheel;
			enc_rightWheel=now_enc_rightWheel;
			//删除已经处理的数据
			if(!cs->buffer_rev.empty())
			{
				cs->buffer_rev.pop_front();
			}
			
		
		pthread_mutex_unlock(&cs->msgMute);

		ros::spinOnce();

		loop_rate.sleep();
	}

  	return 0;
}



