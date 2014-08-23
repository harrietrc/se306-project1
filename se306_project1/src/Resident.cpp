#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"

#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "Resident.h"

using namespace std;
/**
 *	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
 *	@note Rounding is used to calculate the current angle. This approximation is accounted for by using threshholds when processing angles.
 *	@param msg Odometry message from odom topic
 */
void Resident::StageOdom_callback(nav_msgs::Odometry msg)
{
	//a++;
	//Converting from quaternion to radians
	currentAngle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		currentAngle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}

	//Rounding to 3 decimal places
	currentAngle = ((int)(currentAngle * 1000 + .5) / 1000.0);

	//	currentAngle = currentAngle * (180 / M_PI);

	//	if (count % 10 == 0){
	//		ROS_INFO ("px %f", msg.pose.pose.position.x);
	//		ROS_INFO ("py %f", msg.pose.pose.position.y);
	//		ROS_INFO ("angle %f", currentAngle);
	//	}

	px = msg.pose.pose.position.x;
	py = msg.pose.pose.position.y;

	if (isMoving == true){
		move();
	}


}



/**
 *	@brief Callback function to process laser scan messsages.
 *	You can access the range data from msg.ranges[i]. i = sample number
 *	@note Currently blank as it is not in use. Navigation operates through a checkpoint system.
 *	@param msg Single scan from a planar laser range finder
 */
void Resident::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number

}

/**
 *	@brief Main function for the Resident process.
 *	Controls node setup and periodic events.
 */
int Resident::run(int argc, char *argv[]) {

	pair<double, double> c1 = make_pair(40,30);
	pair<double, double> c2 = make_pair(30,40);

	shortestPath.push_back(c1);
	shortestPath.push_back(c2);

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");
	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);


	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Resident::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Resident::StageLaser_callback, this);

	//ros::Rate loop_rate(10);
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
		//ROS_INFO("Hunger %d",hunger);
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);


		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}

/**
 *	@brief Redirects to main function (run()) of the node.
 */
int main(int argc, char **argv) {
	Resident *a = new Resident();
	a->Resident::run(argc, argv);
}
