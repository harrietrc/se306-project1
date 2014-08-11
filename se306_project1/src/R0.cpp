#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

double goal_x;
double goal_y;
double goal_theta;


//pose of the robot
double px;
double py;
double theta;

double cur_angle;
double init_vector_x;
double init_vector_y;
double goal_vector_x;
double goal_vector_y;
double dot;

double w;


void StageOdom_callback(nav_msgs::Odometry msg)
{
	
	//Converting from quaternion to radians
	cur_angle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		cur_angle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}	

	//ROUNDINGSHIT cur_angle = ((int)(cur_angle * 10 + .5) / 10.0);
	
	//When the robot is facing the correct direction, start moving
	if (goal_theta == cur_angle) {
		angular_z = 0;
		linear_x = 0.2;
	}
	
	//When robot reaches the goal, stop moving
	if ((px == goal_x) && (py == goal_y)) {	
		linear_x = 0;
	
	}

}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
}

int main(int argc, char **argv)
{

 //initialize robot parameters

	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	px = 10;
	py = 10;
	cur_angle = 0;

	//Goal 
	goal_x = 0;
	goal_y = 0;

	//Finding the vector that the robot is facing and the goal vector
	init_vector_x = cos(cur_angle);
	init_vector_y = sin(cur_angle);
	goal_vector_x = goal_x - px;
	goal_vector_y = goal_y - py;

	//Finding dot product between init_vector and goal_vector
	dot = (goal_vector_x*init_vector_x) + (goal_vector_y*init_vector_y);	
	
	//Calculating angle for robot to face to the goal
	goal_theta = acos(dot/sqrt(pow(goal_vector_x,2) + pow(goal_vector_y,2)));
	//rounding goal_theta to two decimal places
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0.2;

	goal_x = goal_x - px;
	goal_y = goal_x - py;  
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode0");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

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
		ROS_INFO("dude");
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

return 0;

}
