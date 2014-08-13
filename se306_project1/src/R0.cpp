#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"
#include "time_conversion.hpp"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 5 + msg.pose.pose.position.x;
	py =10 + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", px);
	ROS_INFO("Current y position is: %f", py);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

/* 
	Model for all periodic events - start time, stop time, period. The issue with this at the moment is that
	any behaviour defined here will occur at the end of the duration provided when creating the timer. This
	could easily solved if this method had access to the value of that duration - I'll look into solving this
	later. It's more of a usabiity issue than a usefulness one, as you can  adjust the startTime and endTime
	but I'd prefer it to be more intuitive.
	NOT CURRENTLY SUPPORTED: Events on certain days (other than every x number of days), non-periodic events 
							 during the defined period (startTime - endTime)
*/
void testCallback1(const ros::TimerEvent&) {
	int startTime = time_conversion::simHoursToRealSecs(6); // Start callback at 6am
	int endTime = time_conversion::simHoursToRealSecs(12); // Stop callback at 12pm

	int tnow = ros::Time::now().toSec(); // The simulation time now
	int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

	// Behaviour should only occur if the simulation time is between the specified start and end times.
	if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) { // Note that this will run at the end of the duration specified for the timer.
		ROS_INFO("providing medication: "); 
		std::ostringstream s;
		s << tnow; 
		ROS_INFO(s.str().c_str()); // Just shows elapsed seconds
		ros::Duration(5).sleep();
	}
}

void testCallback2(const ros::TimerEvent&) {
	int startTime = time_conversion::simHoursToRealSecs(0); // Start callback at 6am
	int endTime = time_conversion::simHoursToRealSecs(24); // Stop callback at 12pm

	int tnow = ros::Time::now().toSec(); // The simulation time now
	int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

	if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) {
		ROS_INFO("visiting resident: ");
		std::ostringstream s;
		s << tnow; 
		ROS_INFO(s.str().c_str());
	}
}

int main(int argc, char **argv)
{

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
	//Initial velocity
	linear_x = 0.2;
	angular_z = 0.2;
	
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

// Timers for event scheduling. Timers that run callback at the same time will have those callbacks queued.
int dur1 = time_conversion::simHoursToRealSecs(4);
ros::Timer visitTimer = n.createTimer(ros::Duration(dur1), testCallback2);
int dur2 = time_conversion::simHoursToRealSecs(2); // Perform callback every 2 simulation hours
ros::Timer medicationTimer = n.createTimer(ros::Duration(dur2), testCallback1); 

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
