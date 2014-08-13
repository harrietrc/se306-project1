#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/DoctorMsg.h"

#include <sstream>
#include "math.h"
#include "Doctor.h"
	
void Doctor::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 5 + msg.pose.pose.position.x;
	py =10 + msg.pose.pose.position.y;
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);
}

void Doctor::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

//custom resident callback function, you get the message object that was sent from Resident
void Doctor::residentStatusCallback(se306_project1::ResidentMsg msg)
{
	// if (msg.health < 20) { // emergency
	// 	Doctor visits()/moves() to the Resident
	// 	if Doctor is next to Resident
	// 		bool healResident = true;
	//		if (msg.health > 20) {
	//			leave() // go outside of house?
	// else
	// 	healResident = false;
	
}

int Doctor::run(int argc, char **argv)
{
	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
	//Initial velocity
	linear_x = 0.2;
	angular_z = 0.2;

	//boolean to indicate whether doctor should heal the resident
	healResident = false;
		
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Doctor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	// Resident subscribes to this topic. Indicates whether Doctor should heal Resident
	ros::Publisher doctor_pub = n.advertise<se306_project1::DoctorMsg>("healResident",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Doctor::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Doctor::StageLaser_callback, this);

	//custom Resident subscriber to "resident/state"
	//ros::Subscriber Resident_sub = n.subscribe<std_msgs::String>("residentStatus",1000,residentStatusCallback);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Doctor::residentStatusCallback, this);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
			
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);

		//setup and publish a doctor msg to resident
		se306_project1::DoctorMsg msg;
		msg.healResident = healResident;
		doctor_pub.publish(msg);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}

/* 
	Redirects to main function (run()) of the node.
*/
int main(int argc, char **argv) {
	Doctor *a = new Doctor;
	a->Doctor::run(argc, argv);
}
