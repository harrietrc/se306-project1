#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"
#include "Visitor.h"
#include "Agent.h"


/**
*	@brief Function that decides what tasks the visitor must do.
**	@note
*	@param Timer reference, determines when the visitors will arrive
*/
void Visitor::delegate(const ros::TimerEvent&)
{
	int startTime = time_conversion::simHoursToRealSecs(6); // Start callback at 6am
	int endTime = time_conversion::simHoursToRealSecs(12); // Stop callback at 12pm

	int tnow = ros::Time::now().toSec(); // The simulation time now
	int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

	// Behaviour should only occur if the simulation time is between the specified start and end times.
	if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) { // Note that this will run at the end of the duration specified for the timer.
		meetResident();
	}
}

/**
*	@brief Visitor goes to the resident's location.
*	@return Returns true if behaviour was successful, and false otherwise.
*/
bool Visitor::meetResident(se306_project1::ResidentMsg msg) {

	//Move to the resident's location;
	if ((sqrt(pow((msg.x - px),2) + pow((msg.y - py),2)) < 2.5)){
		//Move to the residents location. Thats it.
	}
	return true;
}

/**
*	@brief Main function for the Visitor process.
*	Controls node setup and periodic events.
*/
int Visitor::run(int argc, char *argv[])
{
	/* -- Initialisation -- */
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Visitor");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	ros::Rate loop_rate(10);


	/* -- Publish / Subscribe -- */

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Visitor::StageOdom_callback,this);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Visitor::delegate, this);

	// Periodic callback
	int dur2 = time_conversion::simHoursToRealSecs(2); // Perform callback every 2 simulation hours
	ros::Timer VisitorSchedule = n.createTimer(ros::Duration(dur2), &Visitor::delegate, this); 


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
	Visitor *a = new Visitor();
	a->Visitor::run(argc, argv);
}

