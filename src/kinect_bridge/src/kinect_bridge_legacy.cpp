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


using namespace std;

 std::vector<std::vector<double>> pose{ {0 , 0  ,  0}, //0
					  {0 , 0  ,  0}, //1  
					  {0 , 8 , 54}, //2 = left shoulder
					  {0 , 18 , 54}, //3 = left elbow
					  {12, 18 , 54}, //4 = left hand
					  {0 , -8 , 54}, //5 = right shoulder
					  {0 , -18, 54}, //6 = right elbow
					  {12, -18, 54}, //7 = right hand
					  {0 , 0  ,  0}, //8
					  {0 , 0  ,  0}, //9
					  {0 , 0  ,  0}, //10
					  {0 , 0  ,  0}, //11
					  {0 , 0  ,  0}, //12
					  {0 , 0  ,  0}, //13
					  {0 , 0  ,  0}, //14
					  {0 , 0  ,  0} };//15
 std::vector<string> names =    {"rs1", "rs2", "re1", "re2", "rw1", "rw2", "ls1", "ls2", "le1", "le2", "lw1", "lw2"}; 
 std::vector<double> positions = { 0.0 ,  0.0,   0.0 ,  0.0 ,  0.0,   0.0,   0.0,   0.0,  0.0,   0.0,   0.0,   0.0};


void getPose(const kinect_v2::BodyJoints &user_i){
	ROS_INFO("getting pose");
	for(int i = 0; i<16; i++){
		pose[i][0] = user_i.joints[i].position.x;
		pose[i][1] = user_i.joints[i].position.y;
		pose[i][2] = user_i.joints[i].position.z;	
	}
}


void calculateJoints(int user_id){

	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	double dx = 0;
	double dy = 0;
	double dz = 0;

	double r = 0;
	double theta = 0;
	double phi = 0;
	listener.lookupTransform("right_elbow_"+to_string(user_id),"right_shoulder_"+to_string(user_id), ros::Time(0), transform);

	dx = transform.getOrigin().x();
	dy = transform.getOrigin().y();
	dz = transform.getOrigin().z();

	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx);
	phi = acos(dz/r);
		

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[0] = theta; //rs1
	positions[1] = phi;   //rs2

	
	//Right elbow:	"right_wrist" - "right_elbow" --> [dx,dy,dz] --> RE[theta,phi,r] == [re1, re2]
   	listener.lookupTransform("right_hand_"+to_string(user_id),"right_elbow_"+to_string(user_id), ros::Time(0), transform);

	dx = transform.getOrigin().x();
	dy = transform.getOrigin().y();
	dz = transform.getOrigin().z();
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx)+1.57;
	phi = acos(dz/r);

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[2] = theta; //re1
	positions[3] = phi;   //re2


	//Right wrist: 
	positions[4] = 0; //rw1
	positions[5] = 0;   //rw2


	//Left shoulder:	"left_elbow" - "left_shoulder" --> [dx,dy,dz] --> LS[theta,phi,r] == [ls1, ls2]
	listener.lookupTransform("left_elbow_"+to_string(user_id),"left_shoulder_"+to_string(user_id), ros::Time(0), transform);

	dx = transform.getOrigin().x();
	dy = transform.getOrigin().y();
	dz = transform.getOrigin().z();
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx);
	phi = acos(dz/r) + 3.14;
	
	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[6] = theta; //ls1
	positions[7] = phi;   //ls2


	//Left elbow:	"left_wrist" - "left_elbow" --> [dx,dy,dz] --> LE[theta,phi,r] == [le1, le2]
	listener.lookupTransform("left_hand_"+to_string(user_id),"left_elbow_"+to_string(user_id), ros::Time(0), transform);

	dx = transform.getOrigin().x();
	dy = transform.getOrigin().y();
	dz = transform.getOrigin().z();
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx)+1.57;
	phi = acos(dz/r);

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[8] = theta; //le1
	positions[9] = phi;   //le2


	//Left wrist:
	positions[10] = 0; //lw1
	positions[11] = 0;   //lw2


}


void calculateJointPosition(){
	
	double dx = 0;
	double dy = 0;
	double dz = 0;

	double r = 0;
	double theta = 0;
	double phi = 0;
	//Right shoulder:  "right_elbow" - "right_shoulder" --> [dx,dy,dz] --> RS[theta,phi,r] == [rs1, rs2]
	dx = pose[6][0] - pose[5][0];
	dy = pose[6][1] - pose[5][1];
	dz = pose[6][2] - pose[5][2];
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx);
	phi = acos(dz/r);
		

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[0] = theta; //rs1
	positions[1] = phi;   //rs2	

	//Right elbow:	"right_wrist" - "right_elbow" --> [dx,dy,dz] --> RE[theta,phi,r] == [re1, re2]
   	dx = pose[7][0] - pose[6][0];
	dy = pose[7][1] - pose[6][1];
	dz = pose[7][2] - pose[6][2];
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx)+1.57;
	phi = acos(dz/r);

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[2] = theta; //re1
	positions[3] = phi;   //re2


	//Right wrist: 
	positions[4] = 0; //rw1
	positions[5] = 0;   //rw2


	//Left shoulder:	"left_elbow" - "left_shoulder" --> [dx,dy,dz] --> LS[theta,phi,r] == [ls1, ls2]
	dx = pose[3][0] - pose[2][0];
	dy = pose[3][1] - pose[2][1];
	dz = pose[3][2] - pose[2][2];
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx);
	phi = acos(dz/r) + 3.14;
	
	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[6] = theta; //ls1
	positions[7] = phi;   //ls2


	//Left elbow:	"left_wrist" - "left_elbow" --> [dx,dy,dz] --> LE[theta,phi,r] == [le1, le2]
	dx = pose[4][0] - pose[3][0];
	dy = pose[4][1] - pose[3][1];
	dz = pose[4][2] - pose[3][2];
	
	r = sqrt(pow(dx,2.0) +pow(dy,2) + pow(dz,2));
	theta = atan2(dy,dx)+1.57;
	phi = acos(dz/r);

	if(isnan(theta)){
		theta = 1;
	}
	if(isnan(phi)){
		phi = 1;	
	}

	positions[8] = theta; //le1
	positions[9] = phi;   //le2


	//Left wrist:
	positions[10] = 0; //lw1
	positions[11] = 0;   //lw2
	

}


const string get_key_name(int n);
void publishTransform(const kinect_v2::BodyJoints &user_i, int user_id, int j);

void callback_0(const kinect_v2::BodyJoints &user_0)
{
	getPose(user_0);	
	for (int i = 0; i < 16; ++i)
		publishTransform(user_0, 0, i);
	calculateJoints(0);
}

void callback_1(const kinect_v2::BodyJoints &user_1)
{
	getPose(user_1);
	for (int i = 0; i < 16; ++i)
		publishTransform(user_1, 1, i);
	calculateJoints(1);
}

void callback_2(const kinect_v2::BodyJoints &user_2)
{
	getPose(user_2);
	for (int i = 0; i < 16; ++i)
		publishTransform(user_2, 2, i);
	calculateJoints(2);
}

void callback_3(const kinect_v2::BodyJoints &user_3)
{
	getPose(user_3);
	for (int i = 0; i < 16; ++i)
		publishTransform(user_3, 3, i);
	calculateJoints(3);
}

void callback_4(const kinect_v2::BodyJoints &user_4)
{	
	getPose(user_4);
	for (int i = 0; i < 16; ++i)
		publishTransform(user_4, 4, i);
	calculateJoints(4);
}

void callback_5(const kinect_v2::BodyJoints &user_5)
{
	getPose(user_5);
	for (int i = 0; i < 16; ++i)
		publishTransform(user_5, 5, i);
	calculateJoints(5);
	
}

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

	ros::Publisher pub_0 = n.advertise<sensor_msgs::JointState>("joint_states", 5);
	
	  tf::TransformBroadcaster br;
  	  tf::Transform transform;

	while(ros::ok()){
	
	tf::Quaternion q;
	q.setRPY(1.57, 0, 1.57);
	q.normalize();
	transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
    	transform.setRotation( q );
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "kinect_v2"));
    

	//calculateJointPosition();	
	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();
	

	msg.name = names;	
	msg.position = positions; 
	ROS_INFO("pose value right shoulder is x=  %f y= %f z = %f ", pose[6][0], pose[6][1], pose[6][2]);
	
	pub_0.publish(msg);	
	
	ros::spinOnce();
	}
	return 0;
}

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





