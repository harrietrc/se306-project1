#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"
#include "Caregiver.h"


/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Assistant should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@note Currently this callback processes only resident hunger, controlling the cooking behaviour. More behaviours 
*	can be implemented later.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
//void Caregiver::delegate(se306_project1::ResidentMsg r_msg, se306_project1::AssistantMsg a_msg) no?
void Caregiver::delegate(se306_project1::ResidentMsg msg)
{
	if (msg.state == care) {
		if (!atResident) {
			move(msg.currentCheckpoint); //to resident

			if (Visitor::visitResident() == true) { // next to resident
				atResident = true;
			}
			
		}

		if (!hasShowered && atResident) {
			hasShowered = shower(msg); //Will return true when shower is completed
		}

		if (hasShowered && !hasExercised) {
			hasExercised = exercise(msg);
		}

		if (hasExercised) {
			move(FrontDoorEast); //leave or something
		}
	}
}

/**
*	@brief Caregiver helps the resident to shower, by taking them to the shower and helping them clean themseld.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::shower(se306_project1::ResidentMsg msg) {


	std::string position;
	position = "Shower";
	move(position);

	if (msg.position = position){
		//showering
		spin();
		return true;
	}
	return false;
}

/**
*	@brief Caregiver helps the resident to do exercise. Will arrive when excercise scheduled.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::exercise(se306_project1::ResidentMsg msg) {
	
	std::string position;
	position = "BedSouthEast";
	move(position);

	if (msg.position == position){
		//exercising
		spin();
		return true;
	}
	return false;
}

void Caregiver::spin() {
	int counter = 0;
	while (counter < 100) {
		counter++;
		angular_z = 2;
	}
	angular_z = 0;
		
}

/**
*	@brief Main function for the Caregiver process.
*	Controls node setup and periodic events.
*/
int Caregiver::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Caregiver");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	//Booleans
	atResident = false;
	hasShowered = false;
	hasExercised = false;


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Caregiver::StageOdom_callback,this);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Caregiver::delegate, this);

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
	Caregiver *a = new Caregiver();
	a->Caregiver::run(argc, argv);
}


