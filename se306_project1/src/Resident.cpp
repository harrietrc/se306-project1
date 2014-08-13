#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "time_conversion.hpp"


//velocity of the robot
double linear_x;
double angular_z;

//goal pose and orientation
double goal_x;
double goal_y;
double goal_angle;


//current pose and orientation of the robot
double px;
double py;
double cur_angle;

int cc = 1; //current_checkpoint = 0;

bool is_called = true; 

std::pair<double,bool> goal_pair;
std::pair<double, double> ret;	

int checkpoints[5][2] = {  
{30, 25}, 
{30, 7}, 
{43, 7},
{30, 7},
{30, 25}  
};

double kitchen[2] = {43,7};
double kitchen_corner[2] = {30, 7};
double home_centre[2] = {30, 25};
double bed_corner[2] = {32, 40};
double toilet[2] = {10,40};

std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
std::pair<double,bool> calc_goal(double goal_x, double goal_y, double cur_angle, double px, double py); 
void StageOdom_callback(nav_msgs::Odometry msg); 


void StageOdom_callback(nav_msgs::Odometry msg)
{
	ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	//double val = 0;

	//Converting from quaternion to radians
	cur_angle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		cur_angle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}

	//Rounding to 3 decimal places
	cur_angle = ((int)(cur_angle * 1000 + .5) / 1000.0);
	
	//Update the current position
	px = msg.pose.pose.position.x + checkpoints[0][0];
	py = msg.pose.pose.position.y + checkpoints[0][1];
	
	//ROS_INFO("angular z: %f", angular_z);
	ROS_INFO("---------------------------------------------");
		ROS_INFO("px: %f", px);
		ROS_INFO("py: %f", py);
		ROS_INFO("goal x: %f", goal_x);
		ROS_INFO("goal y: %f", goal_y);
		ROS_INFO("Current angle: %f", cur_angle);
		ROS_INFO("Goal theta:   %f", goal_angle);
	ROS_INFO("---------------------------------------------");


	//When goal reached
	if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5) && (py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {
	
		if (cc == 4) { //If at last checkpoint
			linear_x = 0;
			
		} else {
			cc++; //Increment checkpoint index
		}
		goal_x = checkpoints[cc][0];
		goal_y = checkpoints[cc][1];

		if (is_called) {
			//Adjusts for delay between publish and subscribe	
			//val = goal_angle;
			goal_pair = calc_goal(goal_x, goal_y, cur_angle, px, py);
			goal_angle = goal_pair.first;
			//if (goal_pair.second == false) {
			//	goal_angle = val + goal_pair.first;
			//} else {
			//	if(goal_pair.first >= val){
			//		goal_angle = 6.283 + val - goal_pair.first;
			//	} else {			
			//		goal_angle = val - goal_pair.first;
			//	}
			//}
			is_called = false;
		}

		if (goal_angle >= 6.283) {
			goal_angle = goal_angle - 6.283;
		}

	} else { //Do this until goal is reached
		
		is_called = true;
		//val = 0;
		ret = move(goal_x, goal_y, cur_angle, goal_angle, px, py);	
		linear_x = ret.first;
		angular_z = ret.second;
	}
}


//Keeps robot moving by changing linear_x and angular_z
std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py) 
{	
 	std::pair <double,double> _ret (0, 0.001); //Return value of move() function, contains linear_x and angular_x. Defines how robot movees between goals.	

	//When the robot is facing the correct direction, start moving

	if (goal_angle == cur_angle) {
		_ret.first = 0.5; //linear_x
		_ret.second = 0; //angular_z
	}
	
	//if (goal_pair.second) {
	//	_ret.second = _ret.second * -1;
	//}

	//When robot reaches the goal, stop moving (with a leeway of 0.4)
	if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5)) {	
		if ((py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {		
			_ret.first = 0; //linear_x
		}
	}

	return _ret; 
}

std::pair <double,bool> calc_goal(double goal_x, double goal_y, double cur_angle, double px, double py) 
{

	std::pair <double,bool> _ret (0, false);

	//Initial and goal vectors used to calculate goal theta
	double init_vector_x;
	double init_vector_y;
	double goal_vector_x;
	double goal_vector_y;
	//double dot;
	double goal_angle;
	double cross = 0;
	bool is_clockwise = false;
	

	//Finding the vector that the robot is facing and the goal vector
	init_vector_x = cos(cur_angle);
	init_vector_y = sin(cur_angle);
	goal_vector_x = goal_x - px;
	goal_vector_y = goal_y - py;

	//Finding dot product between init_vector and goal_vector
	//dot = (goal_vector_x*init_vector_x) + (goal_vector_y*init_vector_y);		

	//Calculating angle for robot to face to the goal
	//goal_angle = acos(dot/sqrt(pow(goal_vector_x,2) + pow(goal_vector_y,2)));

	cross = (init_vector_x * goal_vector_y) - (goal_vector_x * init_vector_y);
	
	if (cross < 0) {
		is_clockwise = true;
	}
	else{
		is_clockwise = false;
	}
	
	//if (goal_vector_x == 0) {
	//	goal_angle = M_PI/2;
	//}else{
	//	goal_angle = atan(abs(goal_vector_y)/abs(goal_vector_x));	
	//}	
	

	//ROS_INFO("Tan Angle: %f", goal_angle);

	//if ((goal_vector_x < 0) && (goal_vector_y <= 0)) {
	//	goal_angle = M_PI - goal_angle;
	//}else if ((goal_vector_x <= 0) && (goal_vector_y > 0)) {
	//	goal_angle = M_PI + goal_angle;
	//}else if ((goal_vector_x > 0) && (goal_vector_y > 0)) {
	//	goal_angle = (2*M_PI) - goal_angle;
	//}

	ROS_INFO("goal vec x: %f", goal_vector_x);
	ROS_INFO("goal vec y: %f", goal_vector_y);

	goal_angle = atan2(goal_vector_y, goal_vector_x); //pi <= goal_angle < -pi
	if (goal_angle < 0) {
		goal_angle = 2 * M_PI + goal_angle; //Remove sign, then add to pi
	} else if (goal_angle == 2 * M_PI) { //New goal angle =>   >0 to 6.283
		goal_angle = 0;
	}
	goal_angle = (2* M_PI) - goal_angle;

	//rounding goal_angle to three decimal places
	goal_angle = ((int)(goal_angle * 1000 + .5) / 1000.0);

	_ret.first = goal_angle;
	_ret.second = is_clockwise;

	return _ret;
}

int main(int argc, char **argv)
{

 //initialize robot parameters

	goal_pair = std::make_pair(0, false);

	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	px = checkpoints[cc-1][0];
	py = checkpoints[cc-1][1];
	cur_angle = 0;

	//Goal 
	goal_x = checkpoints[cc][0];
	goal_y = checkpoints[cc][1];

	goal_pair = calc_goal(goal_x, goal_y, cur_angle, px, py);
	goal_angle = goal_pair.first;	

	//Initial velocity
	linear_x = 0;
	angular_z = 0.001;
	
	//Align local system to global coordinates
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode0");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);

	ros::Rate loop_rate(1000);

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
	
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

return 0;

}