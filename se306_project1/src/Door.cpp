#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include "math.h"
#include "std_msgs/String.h"
#include "Door.h"

/**
 *	@brief Updates the door's x position, y position, and angle to reflect its current pose.
 *	@param msg Odometry message from odom topic
 */
void Door::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = -20.5 + msg.pose.pose.position.x;
	py = -37.5 + msg.pose.pose.position.y;
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);

	// testing opening and closing
	if (opened == false){
		open();
		ROS_INFO("Opening door");
	} else {
		if (opened == true)
			close();
			ROS_INFO("Closing door");
	}
}

/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Door should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Door::delegate(se306_project1::ResidentMsg msg) {
	if (msg.state == "ill" || msg.state == "emergency" || msg.state == "caregiver" || msg.state == "friends") {
		if (opened == false && resStateWhenOpened != msg.state) {
			resStateWhenOpened = msg.state;
			open();
			ROS_INFO("Opening door");
		}
	} else {
		if (opened == true) {
			close();
			ROS_INFO("Closing door");
		}
	}

}

/**
*	@brief opens the Door
*/
void Door::open() {
	//opened = true;
	linear_x = 100;
	double distanceFromCheckpoint = sqrt(pow((-8.5 - px),2) + pow((-37.5 - py),2)); // opened is -8.5 and -37.5
	if (distanceFromCheckpoint < 0.5) {
		linear_x = 0;
		opened = true;
	}
}

/**
*	@brief closes the Door
*/
void Door::close() {
	//opened = false;
	
	// time for door to automatically close
	int tnow = ros::Time::now().toSec(); // The simulation time now
	int wait = tnow + 1; // one hour later
	// make door wait for an hour? before closing
	while (tnow < wait) {
		tnow = ros::Time::now().toSec(); // The simulation time now
	}
	
	linear_x = -100;
	double distanceFromCheckpoint = sqrt(pow((-20.5 - px),2) + pow((-37.5 - py),2)); // closed is -20.5 and -37.5
	if (distanceFromCheckpoint < 0.5) {
		linear_x = 0;
		opened = false;
	}
}

/**
*	@brief Main function for the Door process.
*	Controls node setup and periodic events.
*/
int Door::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	linear_x = 0;
	angular_z = 0;

	px = -20.5;
	py = -37.5;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Door");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_0/odom",1000, &Door::StageOdom_callback, this);

	// Door subscribes to the Resident status topic
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000, &Door::delegate, this);

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
	Door *a = new Door;
	a->Door::run(argc, argv);
}