#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <sstream>
#include "math.h"
#include "time_conversion.hpp"
#include "Assistant1.h"
#include <cmath>
#include "se306_project1/AssistantMsg.h"
#include "se306_project1/ResidentMsg.h"
//#include "se306_project1/AssistantCommMsg.h"

using namespace std;







/*	@brief Callback function that unpacks and processes resident status messages.
* 	Calls other callback (do) functions.
*	Assistant should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Assistant1::clean(se306_project1::ResidentMsg msg) {
	move("Assistant2Origin");
	if (g.getCheckpointName(currentCheckpoint) == "Assistant2Origin" ) {
		shortestPath.push_back(g.getCoords("CentreStool"));
		shortestPath.push_back(g.getCoords("NextToCentreStool"));
		shortestPath.push_back(g.getCoords("Assistant2Origin"));

		isMoving = true;

	}
}

/**
*	@brief Main function for the Assistant process.
*	Controls node setup and periodic events.
*/
int Assistant1::run(int argc, char **argv)
{
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Assistant1");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	px = 32;
	py = 18;

	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000);
	Assistant_state_pub = n.advertise<se306_project1::AssistantMsg>("assistantStatus", 1000);
	//Assistant_comm_pub = n.advertise<se306_project1::AssistantCommMsg>("assistantComms", 1000);


	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_2/odom",1000, &Assistant1::StageOdom_callback, dynamic_cast<Agent*>(this));
	ros::Subscriber residentSub = n.subscribe("residentStatus",1000, &Assistant1::clean, this);

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
	}

	return 0;

}

/**
*	@brief Redirects to main function (run()) of the node.
*/
int main(int argc, char *argv[]) {
	Assistant1 *a = new Assistant1();
	//setOriginName(argc, argv[0]); // Set the name of the starting checkpoint
	a->Assistant1::run(argc, argv);
}
