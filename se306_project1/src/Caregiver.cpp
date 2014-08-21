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
void Caregiver::delegate(const ros::TimerEvent&)
{
	int startTime = time_conversion::simHoursToRealSecs(6); // Start callback at 6am
	int endTime = time_conversion::simHoursToRealSecs(12); // Stop callback at 12pm

	int tnow = ros::Time::now().toSec(); // The simulation time now
	int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

	// Behaviour should only occur if the simulation time is between the specified start and end times.
	if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) { // Note that this will run at the end of the duration specified for the timer.
		doEatSupport(r_msg);
		doExerciseSupport(r_msg);
		doShowerSupport(r_msg);
		doMoralSupport(r_msg);
	}
	// if (r_msg.hygiene < 50) { // Resident is in dire need of a shower
	// 	doShowerSupport(r_msg);
	// }
	// else if (r_msg.mood <50} { // Resident is demoralized, give him his depression medication
	// 	doMoralSupport(r_msg);
	// }
	// else if (r_msg.fitness) { // Resident needs excercise
	// 	doExerciseSupport(r_msg);
	// }
	// else if (a_msg.FoodDelivered) {
	// 	doEatSupport(r_msg)
	// }
}

/**
*	@brief Caregiver helps the resident to eat. Will feed him after his meal has been prepared by an assistant.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doEatSupport(se306_project1::ResidentMsg msg) {

	if(needs_food){
		needs_food = false;
		//Caregiver should move to residents position in order to "feed" him
	}

	if ((sqrt(pow((msg.x - px),2) + pow((msg.y - py),2)) < 2.5) && msg.hunger == 100){
		//feed complete, now b (leave the house)
		return true;
	}
	return false;
}

/**
*	@brief Caregiver helps the resident to do exercise. Will arrive when excercise scheduled.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doExerciseSupport(se306_project1::ResidentMsg msg) {
	
	if(needs_excercise){
		needs_excercise = false;
		//Caregiver should move to residents position, then help him excercise. Go to gym area etc
	}

	if ((sqrt(pow((msg.x - px),2) + pow((msg.y - py),2)) < 2.5) && msg.excecise == true){
		//excercise complete, leave house
		return true;
	}
	return false;
}

/**
*	@brief Caregiver helps the resident to shower.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doShowerSupport(se306_project1::ResidentMsg msg) {
	
	if(needs_shower){
		needs_shower = false;
		//Caregiver should move to residents position, take him to the shower(spin around) and clean him
	}

	if ((sqrt(pow((msg.x - px),2) + pow((msg.y - py),2)) < 2.5) && msg.hygiene == 100){
		//shower complete, take resident to previous position and leave house
		return true;
	}
	return false;
}

/**
*	@brief Caregiver gives the resident moral support.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Caregiver::doMoralSupport(se306_project1::ResidentMsg msg) {
	//No idea what this is. Go hug him or something
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
	ros::init(argc, argv, "Caregiver");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	//Booleans which determine if the caregiver should care for the resident
	needs_food = false;
	needs_excercise = false;
	needs_shower = false;
	needs_moral_support = false;


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Caregiver::StageOdom_callback,this);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Caregiver::delegate, this);


	ros::Subscriber assitant_sub = n.subscribe<se306_project1::AssistantMsg>("assistantStatus",1000,&Caregiver::delegate, this);

	// Periodic callback
	int dur2 = time_conversion::simHoursToRealSecs(2); // Perform callback every 2 simulation hours
	ros::Timer caregiverSchedule = n.createTimer(ros::Duration(dur2), &Caregiver::delegate, this); 


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

