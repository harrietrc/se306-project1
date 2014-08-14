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
	
//velocity of the robot
double linear_x;
double angular_z;

//goal pose and orientation
double goal_x;
double goal_y;
double goal_angle;
bool isSet = false;

//current pose and orientation of the robot
double px;
double py;
double cur_angle;

int cc = 1; //current_checkpoint = 0;

std::pair<double, double> ret;	

std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py); 
void StageOdom_callback(nav_msgs::Odometry msg); 

			int checkpoints[4][2] = {
				{10, -7},
				{10, 1},
				{30, 20},
				{30, 25}
				};
	
void Doctor::StageOdom_callback(nav_msgs::Odometry msg)
{
	//ret = std::make_pair(0, 0); //initialize pair. Used to get return.

	////Converting from quaternion to radians
	//cur_angle = acos(msg.pose.pose.orientation.w) * 2;
	//if (msg.pose.pose.orientation.z > 0) {
		//cur_angle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	//}

	////Rounding to 3 decimal places
	//cur_angle = ((int)(cur_angle * 1000 + .5) / 1000.0);
	
	////Update the current position
	//px = msg.pose.pose.position.x + checkpoints[0][0];
	//py = msg.pose.pose.position.y + checkpoints[0][1];
	
	////When goal reached
	//if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5) && (py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {
	//isSet = false;
		//if (cc == 8) { //If at last checkpoint
			//linear_x = 0;
		//} else {
			//cc++; //Increment checkpoint index
		//}
		//goal_x = checkpoints[cc][0];
		//goal_y = checkpoints[cc][1];
	
		////Account for delay by subtracting delay values from current pose and orientation
		//goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle - M_PI/20, px - 0.1, py - 0.1);

	//} else { //Do this until goal is reached
		//ret = move(goal_x, goal_y, cur_angle, goal_angle, px, py);	
		//linear_x = ret.first;
		//angular_z = ret.second;
	//}
}


//Keeps robot moving by changing linear_x and angular_z
std::pair<double, double> Doctor::move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py) 
{		
	
	std::pair<double,double>_ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	double moveSpeed = M_PI/2;
	moveSpeed = ((int)(moveSpeed * 1000 + .5) / 1000.0);

	//When the robot is facing the correct direction, start moving
	double threshold = cur_angle-moveSpeed/10;
	threshold = ((int)(threshold * 1000 + .5) / 1000.0);

	if ((goal_angle  == threshold) || isSet) {
		_ret.first = 5; //linear_x
		_ret.second = 0; //angular_z
		isSet = true;
	} else if ((goal_angle <= cur_angle + 0.3) && (goal_angle >= cur_angle - 0.3) )  {
		_ret.first = 0; //linear_x
		_ret.second = fabs(goal_angle - cur_angle); //angular_z
		if (goal_angle == cur_angle) {
			isSet = true;		
		}

	} else {
		_ret.first = 0; //linear_x
		_ret.second = moveSpeed; //angular_z
		isSet = false;
	}


	if ((px-0.1 <= goal_x + 0.5) && (px-0.1 >= goal_x - 0.5) && (py-0.1 <= goal_y + 0.5) && (py-0.1 >= goal_y - 0.5)) {	
			_ret.first = 0; 
			isSet = false;
	}

	return _ret; 
}

double Doctor::calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py) 
{

	////Initial and goal vectors used to calculate goal theta
	//double init_vector_x;
	//double init_vector_y;
	//double goal_vector_x;
	//double goal_vector_y;
	//double goal_angle;
	
	////Finding the vector that the robot is facing and the goal vector
	//init_vector_x = cos(cur_angle);
	//init_vector_y = sin(cur_angle);
	//goal_vector_x = goal_x - px;
	//goal_vector_y = goal_y - py;
	
	//goal_angle = atan2(goal_vector_y, goal_vector_x); //pi <= goal_angle < -pi
	//if (goal_angle < 0) {
		//goal_angle = 2 * M_PI + goal_angle; //Remove sign, then add to pi
	//} else if (goal_angle == 2 * M_PI) { //New goal angle =>   >0 to 6.283
		//goal_angle = 0;
	//}
	//goal_angle = (2* M_PI) - goal_angle;

	////rounding goal_angle to three decimal places
	//goal_angle = ((int)(goal_angle * 1000 + .5) / 1000.0);
	
	//return goal_angle;
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
	if (msg.health < 20 && healResident == false) // emergency
	{
		healResident = true;
		ROS_INFO("Resident is in critical condition (EMERGENCY)");
		ROS_INFO("Resident health is: %d", msg.health);
	} else if (msg.health >= 20)
	{
		healResident = false;
	}
	
}

int Doctor::run(int argc, char **argv)
{
	////Initial pose. This is the same as the pose used in the world file.
	//px = checkpoints[cc-1][0];
	//py = checkpoints[cc-1][1];
	//cur_angle = 0;

	////Set goal pose 
	//goal_x = checkpoints[cc][0];
	//goal_y = checkpoints[cc][1];

	//goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle, px, py);

	//Initial velocities
	linear_x = 0;
	angular_z = 0;

	//boolean to indicate whether doctor should heal the resident
	healResident = false;
		
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Doctor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000); 

	// Resident subscribes to this topic. Indicates whether Doctor should heal Resident
	ros::Publisher doctor_pub = n.advertise<se306_project1::DoctorMsg>("healResident",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_2/odom",1000, &Doctor::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_2/base_scan",1000,&Doctor::StageLaser_callback, this);

	//custom Resident subscriber to "resident/state"
	//ros::Subscriber Resident_sub = n.subscribe<std_msgs::String>("residentStatus",1000,residentStatusCallback);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Doctor::residentStatusCallback, this);

	ros::Rate loop_rate(1000);

	//a count of howmany messages we have sent
	int count = 0;
	int i =0;

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
		
		if (healResident == true && count % 10 == 0){
					i += 1;
					if (i == 10)
					{
						doctor_pub.publish(msg);
						ROS_INFO("Doctor used heal on Resident");
						i = 0;
					}
		}

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
