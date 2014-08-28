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

using namespace std;
//PriorityQueue *status_queue = PriorityQueue::getInstance();

/**
*	@brief Gets current resident status and publishes it to a topic for assistants and doctors/nurses.
*	May convert to string and publish in the standard way - do we need custom messages any more?
*/
void Resident::publishStatus() {

}

/**
*	@brief Makes the resident go to bed and sleep, as scheduled by a timer.
*/
bool Resident::doSleep(const ros::TimerEvent&) {
	return true;
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
	if (msg.data == "Done") { // friend has finished talking with Resident
		boredom = 0;
	}
}

/**
*	@brief Increases the Resident's hunger (towards full) if food has been delivered.
*	@param msg A custom message from an Assistant robot. 
*/
void Resident::assistant_callback(se306_project1::AssistantMsg msg)
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

	//priorityQueue myQueue;
	//	myQueue.addToPQ(bored);

	shortestPath.push_back(c1);
	shortestPath.push_back(c2);

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_0/odom",1000, &Agent::StageOdom_callback, dynamic_cast<Agent*>(this));
	
	// Resident subscribes to this topic by Doctor
	ros::Subscriber doctor_sub = n.subscribe<se306_project1::DoctorMsg>("doctorStatus", 1000, &Resident::doctor_callback, this);
	
	// Resident subscribes to this topic by Friend
	ros::Subscriber friend_sub = n.subscribe("visitorConvo", 1000, &Resident::friend_callback, this);

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
int main(int argc, char **argv) {
	Resident *a = new Resident();
	a->Resident::run(argc, argv);
}
