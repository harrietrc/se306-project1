#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "math.h"
#include "Caregiver.h"
#include "se306_project1/ResidentMsg.h"

/**
*	@brief Caregiver helps the resident to eat.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doEatSupport() {
	return true;
}

/**
*	@brief Caregiver helps the resident to do exercise.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doExerciseSupport() {
	return true;
}

/**
*	@brief Caregiver helps the resident to shower.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doShowerSupport() {
	return true;
}

/**
*	@brief Caregiver gives the resident moral support/
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doMoralSupport() {
	return true;
}

/**
*	@brief Main function for the Caregiver process.
*	Controls node setup and periodic events.
*/
int Caregiver::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Relative");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_0/odom",1000, &Agent::StageOdom_callback, dynamic_cast<Agent*>(this));

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
	Caregiver *a = new Caregiver();
	a->Caregiver::run(argc, argv);
}
