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
#include <se306_project1/src/Agent.h>
#include "Visitor.h"

/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Doctor should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Doctor::delegate(se306_project1::ResidentMsg msg) {
	
}

/**
*	@brief Medicates/heals/diagnoses resident, improving their health.
*	@returns true if behaviour is successful.
*/
bool Doctor::doHeal() {
	return true;
}

/**
*	@brief Takes the resident to hospital 
*	(non-doctor-induced emergency, despite the function name...)
*	@returns true if behaviour is successful.
*/
bool Doctor::doHospitalise() {
	return true;
}

/**
*	@brief Main function for the Doctor process.
*	Controls node setup and periodic events.
*/
int Doctor::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Doctor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Agent::StageOdom_callback,this);

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	

	while (ros::ok())
	{
		//messages to stage
		//RobotNode_cmdvel.linear.x = linear_x;
		//RobotNode_cmdvel.angular.z = angular_z;
			
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
	Doctor *a = new Doctor;
	a->Doctor::run(argc, argv);
}
