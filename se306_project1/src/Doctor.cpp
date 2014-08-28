#include "ros/ros.h"
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

/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Doctor should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Doctor::delegate(se306_project1::ResidentMsg msg) {
	if (msg.state == "ill") { // ill
		doHeal();
	}

	if (msg.state == "sill") { // seriously ill - emergency
		doHospitalise();
	}
}

/**
*	@brief Medicates/heals/diagnoses resident, improving their health.
*	@returns true if behaviour is successful.
*/
bool Doctor::doHeal() {
	
	if (healing == false && readyToHeal == false) {
		ROS_INFO("Doctor is on the way"); // print once that the doctor is coming
		healing = true;
	}
	if (healing == true && readyToHeal == false) {
		if (Visitor::visitResident() == true) { // next to resident
			readyToHeal = true;
		}
	}

	if (readyToHeal == true && healing == true) { // Resident should be healed by now so go back outside
		healing = false;
		// move(<outsideHouse>);
	}
	
	return readyToHeal; // This method doesn't have to return a bool but idk
}

/**
*	@brief Takes the resident to hospital 
*	(non-doctor-induced emergency, despite the function name...)
*	@returns true if behaviour is successful.
*/
bool Doctor::doHospitalise() {

	if (hospitalise == false && hospitalise == false) {
		// behaviour is that when the resident is seriously ill, the doctor including two nurses will come to the resident
		// once the doctor is next to the resident, they will all move together back outside of the house
		ROS_INFO("Doctor and nurses are on the way to take Resident to the hospital"); // print once that the doctor is coming
		hospitalise = true;
	}
	if (hospitalise == true && readyToHospitalise == false) {
		if (Visitor::visitResident() == true) { // next to resident
			readyToHospitalise = true;
		}
	}
	
	if (readyToHospitalise == true && hospitalise == true) { // go back outside once Doctor is next to resident
		hospitalise = false;
		// move(<outsideHouse>);
	}
	
	return readyToHospitalise; // This method doesn't have to return a bool but idk
}

/**
*	@brief Main function for the Doctor process.
*	Controls node setup and periodic events.
*/
int Doctor::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	//healing = false;
	//readyToHeal = false;
	//hospitalise = false;
	//readyToHospitalise = false;
	
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
	ros::Subscriber StageOdo_sub = n.subscribe("robot_0/odom",1000, &Agent::StageOdom_callback, dynamic_cast<Agent*>(this));

	// Doctor subscribes to the Resident status topic
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000, &Doctor::delegate, this);
	
	// Resident subscribes to this topic.
	ros::Publisher doctor_pub = n.advertise<se306_project1::DoctorMsg>("healResident",1000);

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	se306_project1::DoctorMsg dMsg; // Doctor message to resident

	while (ros::ok())
	{
		//messages to stage
		//RobotNode_cmdvel.linear.x = linear_x;
		//RobotNode_cmdvel.angular.z = angular_z;
			
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		/*
		* Doctor publishes these messages once when the doctor is ready (next to the resident)
		* heal = heal resident if true
		* hospitalise = hospitalise resident if true
		*/
		if (readyToHeal == true) {
			dMsg.heal = true;
			dMsg.hospitalise = false;
			readyToHeal = false;
			doctor_pub.publish(dMsg);
		} else if (readyToHospitalise == true) {
			dMsg.heal = false;
			dMsg.hospitalise = true;
			readyToHospitalise = false;
			doctor_pub.publish(dMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/**
*	@brief Redirects to main function (run()) of the node.
*/
int main(int argc, char *argv[]) {
	Doctor *a = new Doctor();
	a->Doctor::run(argc, argv);
}
