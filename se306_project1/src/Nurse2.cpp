#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include <sstream>
#include "math.h"
#include "Nurse2.h"

std::string originName = "Nurse2Origin";

/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Nurse should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Nurse2::delegate(se306_project1::ResidentMsg msg) {
	if (msg.state == "sill") { // seriously ill - emergency
		doHospitalise();
	} else {
		readyToHospitalise = false;
	}
}

/**
*	@brief Takes the resident to hospital 
*	(non-doctor-induced emergency, despite the function name...)
*/
void Nurse2::doHospitalise() {

	double lastCheckpointX = shortestPath.at(shortestPath.size()-1).first;
	double lastCheckpointY = shortestPath.at(shortestPath.size()-1).second;

	double distanceFromCheckpoint = sqrt(pow((lastCheckpointX - px),2) + pow((lastCheckpointY - py),2));
	
	if (readyToHospitalise == false) {
		// move(<toResident>);
		if (distanceFromCheckpoint < 0.5) { // next to resident
			readyToHospitalise = true;
		}
	}
	
	if (readyToHospitalise == true) { // go back outside once Nurse is next to resident
		// move(<outsideHouse>);
	}

}

/**
*	@brief Main function for the Nurse process.
*	Controls node setup and periodic events.
*/
int Nurse2::run(int argc, char *argv[])
{

	/* -- Initialisation -- */
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Nurse2");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	// Nurse subscribes to the Resident status topic
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000, &Nurse2::delegate, this);

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
	Nurse2 *a = new Nurse2();
	a->Nurse2::run(argc, argv);
}