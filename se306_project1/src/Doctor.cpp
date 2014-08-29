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
	if (msg.state == "healthLow") { // ill
		doHeal(msg);
	}else if (msg.state == "emergency") { // seriously ill - emergency
	}else{
		move("DoctorOrigin");
	}
}

/**
*	@brief Medicates/heals/diagnoses resident, improving their health.
*	@returns true if behaviour is successful.
*/
void Doctor::doHeal(se306_project1::ResidentMsg msg) {

	double distanceFromCheckpoint = sqrt(pow((msg.currentCheckpointX - px),2) + pow((msg.currentCheckpointY - py),2));

		move(msg.currentCheckpoint);

		if (distanceFromCheckpoint < 5) {
			ROS_INFO("Yes");
			stopMoving();
			isHealed == true;
			isFacingCorrectly = false;

			se306_project1::DoctorMsg dmsg;
			dmsg.ResidentHealed = true;
			Doctor_state_pub.publish(dmsg);

			currentCheckpoint.first = msg.currentCheckpointX;
			currentCheckpoint.second = msg.currentCheckpointY;
			//move("Assistant1Origin");
			isHealed == false;
			move("DoctorOrigin");

		}
}

/**
*	@brief Takes the resident to hospital 
*	(non-doctor-induced emergency, despite the function name...)
*	@returns true if behaviour is successful.
*/
void Doctor::hospitalise(se306_project1::ResidentMsg msg) {


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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);

	Doctor_state_pub = n.advertise<se306_project1::DoctorMsg>("doctorStatus",1000);



	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe("robot_4/odom",1000, &Agent::StageOdom_callback, dynamic_cast<Agent*>(this));

	// Doctor subscribes to the Resident status topic
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000, &Doctor::delegate, this);
	
	// Resident subscribes to this topic.
	ros::Publisher doctor_pub = n.advertise<se306_project1::DoctorMsg>("doctorStatus",1000);

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	se306_project1::DoctorMsg dMsg; // Doctor message to resident

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
			
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		/*
		* Doctor publishes these messages once when the doctor is ready (next to the resident)
		* heal = heal resident if true
		* hospitalise = hospitalise resident if true
		*/
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
