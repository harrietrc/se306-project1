#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/DoctorMsg.h"
#include "se306_project1/AssistantMsg.h"
#include "se306_project1/ResidentMsg.h"

#include "std_msgs/String.h"
#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "Resident.h"
#include "priorityQueue.h"
#include "time_conversion.hpp"

using namespace std;

//PriorityQueue *status_queue = PriorityQueue::getInstance();

/**
*	@brief Gets current resident status and publishes it to a topic for assistants and doctors/nurses.
*	May convert to string and publish in the standard way - do we need custom messages any more?
*/
void Resident::publishStatus(ros::Publisher Resident_state_pub) {


	// Creating a message for residentStatus
	residentState = stateQueue.checkCurrentState();
	se306_project1::ResidentMsg msg;
	//residentState = "medication";    //hardcoded state
	msg.state = residentState;
	msg.currentCheckpoint = g.getCheckpointName(currentCheckpoint);
	msg.currentCheckpointX = currentCheckpoint.first;
	msg.currentCheckpointY = currentCheckpoint.second;
	Resident_state_pub.publish(msg);
}

void Resident::triggerRandomEvents(){

	int randomGeneratedForEmergency = rand() % 100;
	int randomGeneratedForIll = rand() % 100;


	if (randomGeneratedForEmergency <= 3){
		stateQueue.addToPQ(emergency);
		health = 10;
	}

	if (randomGeneratedForIll <= 20){
		health = health - 2;
		if (health < 50){
			stateQueue.addToPQ(healthLow);
		}
	}

}

void Resident::checkStatus(){
	residentState = stateQueue.checkCurrentState();
	if (residentState == "friends"){
		move("ResidentSofa");
	}
	if (residentState == "caregiver"){
		move("BedSouthWest");
	}
	if (hunger > 70){
		stateQueue.addToPQ(hungry);
	}
	if (boredom > 70){
		stateQueue.addToPQ(bored);
	}
}

void Resident::medicationCallback(const ros::TimerEvent&){
	stateQueue.addToPQ(medication);
}
void Resident::hungerCallback(const ros::TimerEvent&){
	stateQueue.addToPQ(hungry);
}
void Resident::caregiverServicesCallback(const ros::TimerEvent&){
	//stateQueue.addToPQ(caregiver);
}
void Resident::caregiverServicesDoneCallback(const ros::TimerEvent&){
	stateQueue.removeState(caregiver);
}
void Resident::wakeCallback(const ros::TimerEvent&){
	stateQueue.removeState(tired);
	ROS_INFO ("Wake up!");
}
void Resident::sleepCallback(const ros::TimerEvent&){
	stateQueue.addToPQ(tired);
}
void Resident::friendsCallback(const ros::TimerEvent&){
	stateQueue.addToPQ(friends);
}
void Resident::friendsDoneCallback(const ros::TimerEvent&){
	stateQueue.removeState(friends);
}


/**
*	@brief Increases the resident's health when the doctor heals them.
*	@param msg A custom message from an Assistant robot.
*	@remarks Perhaps add a delay between medication/diagnosis and healing?
*/
void Resident::doctor_callback(se306_project1::DoctorMsg msg)
{
	if (msg.heal == true) { // at this point Doctor should be next to resident and then doctor should start leaving back to his origin
		health = 100;
	}
	else if (msg.hospitalise == true) { // at this point the doctor + 2 nurses should be next to the resident
		// move(outside house)
		// stay outside for a while?
		// when he returns (pass the door or something)
		//     health = 100;
	}
}

void Resident::friend_callback(const std_msgs::String::ConstPtr& msg)
{
//	if (msg.data == "Done") { // friend has finished talking with Resident
//		boredom = 0;
//	}
}

/**
*	@brief Increases the Resident's hunger (towards full) if food has been delivered.
*	@param msg A custom message from an Assistant robot. 
*/
void Resident::assistant_callback(se306_project1::AssistantMsg msg)
{


	if (msg.FoodDelivered == true) {
		hunger = 100;
		stateQueue.removeState(hungry);
		ROS_INFO("FED");
	}
	if (msg.ResidentMedicated == true) {
		stateQueue.removeState(medication);
		ROS_INFO("Medicated");
	}
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number

}

/**
 *	@brief Main function for the Resident process.
 *	Controls node setup and periodic events.
 */
int Resident::run(int argc, char *argv[]) {


	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	int count = 0;
	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher Resident_state_pub = n.advertise<se306_project1::ResidentMsg>("residentStatus", 1000);

	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	ros::Subscriber assistantSub = n.subscribe("assistantStatus",1000, &Resident::assistant_callback, this);



	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_0/odom",1000, &Agent::StageOdom_callback, dynamic_cast<Agent*>(this));
	
	// Resident subscribes to this topic by Doctor
	ros::Subscriber doctor_sub = n.subscribe<se306_project1::DoctorMsg>("doctorStatus", 1000, &Resident::doctor_callback, this);
	
	// Resident subscribes to this topic by Friend
	ros::Subscriber friend_sub = n.subscribe("visitorConvo", 1000, &Resident::friend_callback, this);

	// Periodic callback
	int wakeup = time_conversion::simHoursToRealSecs(0);
	int caregiverServices = time_conversion::simHoursToRealSecs(0.5);
	int morningMedication = time_conversion::simHoursToRealSecs(5);
	int nightMedication = time_conversion::simHoursToRealSecs(14);
	int morningHungry = time_conversion::simHoursToRealSecs(3);
	int afternoonHungry = time_conversion::simHoursToRealSecs(7);
	int nightHungry= time_conversion::simHoursToRealSecs(12);
	int sleep = time_conversion::simHoursToRealSecs(15);
	int friends = time_conversion::simHoursToRealSecs(9);
	int friendsDone = time_conversion::simHoursToRealSecs(11.5);
	int caregiverServicesDone = time_conversion::simHoursToRealSecs(3);
	ros::Timer wakeUpTimer = n.createTimer(ros::Duration(wakeup), &Resident::wakeCallback,this, true);
	ros::Timer caregiverTimer = n.createTimer(ros::Duration(caregiverServices), &Resident::caregiverServicesCallback, this, true);
	ros::Timer caregiverDoneTimer = n.createTimer(ros::Duration(caregiverServicesDone), &Resident::caregiverServicesDoneCallback, this, true);
	ros::Timer medicationTimer = n.createTimer(ros::Duration(morningMedication), &Resident::medicationCallback, this, true);
	ros::Timer medicationTimer2= n.createTimer(ros::Duration(nightMedication), &Resident::medicationCallback, this, true);
	ros::Timer hungryTimer = n.createTimer(ros::Duration(morningHungry), &Resident::hungerCallback, this, true);
	ros::Timer hungryTimer2 = n.createTimer(ros::Duration(afternoonHungry), &Resident::hungerCallback, this, true);
	ros::Timer hungryTimer3 = n.createTimer(ros::Duration(nightHungry), &Resident::hungerCallback, this, true);
	ros::Timer sleepTimer = n.createTimer(ros::Duration(sleep), &Resident::sleepCallback, this, true);
	ros::Timer friendsTimer = n.createTimer(ros::Duration(friends), &Resident::friendsCallback, this, true);
	ros::Timer friendsDoneTimer = n.createTimer(ros::Duration(friendsDone), &Resident::friendsDoneCallback, this, true);

	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;

		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);

		if (count % 10 == 0){
			if (count % 50 == 0){
				boredom += 2;
				hunger += 1;
			}
			//triggerRandomEvents();
			checkStatus();
			publishStatus(Resident_state_pub);
			residentState = stateQueue.checkCurrentState();
			ROS_INFO("State is: %s",residentState.c_str());
		}

		ros::spinOnce();
		loop_rate.sleep();
		count++;
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
