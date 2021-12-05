#include <ros/ros.h>
#include <string>
#include <cstdio>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinect_v2/BodyJoints.h>
#include <iostream>

#include <vector>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "stdio.h"
#include "stdlib.h" 

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

//########################################## Global Variables #########################

//                      sx sy  sz ex  ey  ez  wx  wy wz  H  
float r_arm_points[10] = {0, 8, 54, 0, 8, 44,  0, 8, 34, 0} ;//{0, 8, 54, 0, 8, 44, 0, 8, 30} ;//

//                       sx  sy  sz ex  ey  ez wx  wy wz   H  
float l_arm_points[10] = {0, -8, 54, 0, -8, 44, 0, -8, 34, 0} ;//{0, 8, 54, 0, 8, 44, 0, 8, 30} ;//





//---------------------------------------------------------------get arm points

void getArmPoints(const kinect_v2::BodyJoints &user_i, int user_id){

	
	//-------------------------------right arm
	MatrixXd tf(4,4);
	tf(0,0) = 1;// 0;
	tf(0,1) = 0;// -1;
	tf(0,2) = 0;// 0;
	tf(0,3) = 0;
	
	tf(1,0) = 0;// 0;
	tf(1,1) = 0;// 0;
	tf(1,2) = -1;// -1;
	tf(1,3) = 0;

	tf(2,0) = 0;// 1;
	tf(2,1) = 1;// 0;
	tf(2,2) = 0;// 0;
	tf(2,3) = 1;

	tf(3,0) = 0;
	tf(3,1) = 0;
	tf(3,2) = 0;
	tf(3,3) = 1;
	
	VectorXd pt(4);
	pt(0) = user_i.joints[6].position.x;
	pt(1) = user_i.joints[6].position.y;
	pt(2) = user_i.joints[6].position.z;
	pt(3) = 1;
	VectorXd npt(4); 
	npt = tf*pt;	

	r_arm_points[0] = npt(1); //right shoulder x
	r_arm_points[1] = -npt(0); //right shoulder y
	r_arm_points[2] = npt(2); //right shoulder z

	pt(0) = user_i.joints[7].position.x;
	pt(1) = user_i.joints[7].position.y;
	pt(2) = user_i.joints[7].position.z;
	pt(3) = 1;
	npt = tf*pt;	

        r_arm_points[3] = npt(1); //right elbow x
        r_arm_points[4] = -npt(0); //right elbow y
        r_arm_points[5] = npt(2); //right elbow z

	pt(0) = user_i.joints[8].position.x;
	pt(1) = user_i.joints[8].position.y;
	pt(2) = user_i.joints[8].position.z;
	pt(3) = 1;
	npt= tf*pt;

	r_arm_points[6] = npt(1); //right wrist x
	r_arm_points[7] = -npt(0); //right wrist y
	r_arm_points[8] = npt(2); //right wrist z

       //------------------------------------------------- LEFT ARM

	
	tf(0,0) = 1;// 0;
	tf(0,1) = 0;// -1;
	tf(0,2) = 0;// 0;
	tf(0,3) = 0;
	
	tf(1,0) = 0;// 0;
	tf(1,1) = 0;// 0;
	tf(1,2) = -1;// -1;
	tf(1,3) = 0;

	tf(2,0) = 0;// 1;
	tf(2,1) = 1;// 0;
	tf(2,2) = 0;// 0;
	tf(2,3) = 1;

	tf(3,0) = 0;
	tf(3,1) = 0;
	tf(3,2) = 0;
	tf(3,3) = 1;
	
	
	pt(0) = user_i.joints[3].position.x;
	pt(1) = user_i.joints[3].position.y;
	pt(2) = user_i.joints[3].position.z;
	pt(3) = 1; 
	npt = tf*pt;	

	l_arm_points[0] = npt(1); //left shoulder x
	l_arm_points[1] = -npt(0); //left shoulder y
	l_arm_points[2] = npt(2); //left shoulder z

	pt(0) = user_i.joints[4].position.x;
	pt(1) = user_i.joints[4].position.y;
	pt(2) = user_i.joints[4].position.z;
	pt(3) = 1;
	npt = tf*pt;	

        l_arm_points[3] = npt(1); //left elbow x
        l_arm_points[4] = -npt(0); //left elbow y
        l_arm_points[5] = npt(2); //left elbow z

	pt(0) = user_i.joints[5].position.x;
	pt(1) = user_i.joints[5].position.y;
	pt(2) = user_i.joints[5].position.z;
	pt(3) = 1;
	npt= tf*pt;

	l_arm_points[6] = npt(1); //left wrist x
	l_arm_points[7] = -npt(0); //left wrist y
	l_arm_points[8] = npt(2); //left wrist z

		
}


//########################################## FUNCTIONS ################################
const string get_key_name(int n)
{
	switch (n)
	{
		case	0:return	"base_spine"; break;
		case	1:return	"neck"; break;
		case	2:return	"head"; break;
		case	3:return	"left_shoulder"; break;
		case	4:return	"left_elbow"; break;
		case	5:return	"left_hand"; break;
		case	6:return	"right_shoulder"; break;
		case	7:return	"right_elbow"; break;
		case	8:return	"right_hand"; break;
		case	9:return	"left_hip"; break;
		case	10:return	"left_knee"; break;
		case	11:return	"left_ankle"; break;
		case	12:return	"right_hip"; break;
		case	13:return	"right_knee"; break;
		case	14:return	"right_ankle"; break;
		case	15:return	"shoulder_spine"; break;
	}
}

void publishTransform(const kinect_v2::BodyJoints &user_i, int user_id, int joint_id)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "kinect_v2";
	transformStamped.child_frame_id = get_key_name(joint_id) + "_" + to_string(user_id);
	transformStamped.transform.translation.x = user_i.joints[joint_id].position.x;
	transformStamped.transform.translation.y = user_i.joints[joint_id].position.y;
	transformStamped.transform.translation.z = user_i.joints[joint_id].position.z;
	transformStamped.transform.rotation.w = user_i.joints[joint_id].orientation.w;
	transformStamped.transform.rotation.x = user_i.joints[joint_id].orientation.x;
	transformStamped.transform.rotation.y = user_i.joints[joint_id].orientation.y;
	transformStamped.transform.rotation.z = user_i.joints[joint_id].orientation.z;

	br.sendTransform(transformStamped);
}

void callback_0(const kinect_v2::BodyJoints &user_0)
{	
	if(user_0.tracked.compare("YES") == 0){
		getArmPoints(user_0, 0);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_0, 0, i);
	}
}

void callback_1(const kinect_v2::BodyJoints &user_1)
{
	if(user_1.tracked.compare("YES") == 0){
		getArmPoints(user_1, 1);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_1, 1, i);
	}
}

void callback_2(const kinect_v2::BodyJoints &user_2)
{
	if(user_2.tracked.compare("YES") == 0){
		getArmPoints(user_2, 2);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_2, 2, i);
	}	
}

void callback_3(const kinect_v2::BodyJoints &user_3)
{
	if(user_3.tracked.compare("YES") == 0){
		getArmPoints(user_3, 3);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_3, 3, i);
	}
}

void callback_4(const kinect_v2::BodyJoints &user_4)
{	
	if(user_4.tracked.compare("YES") == 0){
		getArmPoints(user_4, 4);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_4, 4, i);
	}
}

void callback_5(const kinect_v2::BodyJoints &user_5)
{
	if(user_5.tracked.compare("YES") == 0){
		getArmPoints(user_5, 5);
	}
	for (int i = 0; i < 16; ++i){
		publishTransform(user_5, 5, i);
	}
	
}


void callback_joy(const sensor_msgs::Joy &msg)
{
	r_arm_points[9] = msg.buttons[2];
	l_arm_points[9] = msg.buttons[3];
	
}


//################################################################  MAIN

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_bridge");
	ros::NodeHandle n;
	ros::Subscriber sub_0 = n.subscribe("user_0", 1, &callback_0);
	ros::Subscriber sub_1 = n.subscribe("user_1", 1, &callback_1);
	ros::Subscriber sub_2 = n.subscribe("user_2", 1, &callback_2);
	ros::Subscriber sub_3 = n.subscribe("user_3", 1, &callback_3);
	ros::Subscriber sub_4 = n.subscribe("user_4", 1, &callback_4);
	ros::Subscriber sub_5 = n.subscribe("user_5", 1, &callback_5);
	
	ros::Subscriber hand = n.subscribe("joy",1, &callback_joy);

	ros::Publisher pub_r = n.advertise<std_msgs::Float32MultiArray>("r_arm_points", 10);
	ros::Publisher pub_l = n.advertise<std_msgs::Float32MultiArray>("l_arm_points", 10);	

	tf::TransformBroadcaster br;
  	tf::Transform transform;
	ros::Rate loop_rate(10);

	while(ros::ok()){
//---------------------------------------------world to kinect transform
	tf::Quaternion q;
	q.setRPY(1.57, 0, 1.57);
	q.normalize();
	transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    	transform.setRotation( q );
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "kinect_v2"));
//---------------------------------------------right arm
	std_msgs::Float32MultiArray r_arm;
	for(int i = 0; i < 10; i++){
		r_arm.data.push_back(r_arm_points[i]);	
	}
	pub_r.publish(r_arm);

//---------------------------------------------left arm
	std_msgs::Float32MultiArray l_arm;
	for(int i = 0; i < 10; i++){
		l_arm.data.push_back(l_arm_points[i]);	
	}
	pub_l.publish(l_arm);
		
	
	ros::spinOnce();
	loop_rate.sleep();
	}
	return 0;
}


