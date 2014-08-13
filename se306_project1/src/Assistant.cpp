#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h" // Resident-msg.msg 

#include <sstream>
#include "math.h"
#include "time_conversion.hpp"

class Assistant {

	//velocity of the robot
	double linear_x;
	double angular_z;
	
	//pose of the robot
	double px;
	double py;
	double theta;
	
	// Enumeration of type of robot
	//enum Type{FRIEND, RELATIVE, DOCTOR, NURSE, CAREGIVER, ASSISTANT, RESIDENT};
	
	//int robot_id = 10;
	//int numOfAssistants;
	
	// Enum or string? to be specified
	//String Status;
	
	void StageOdom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		//px = 5 + msg.pose.pose.position.x;
		//py =10 + msg.pose.pose.position.y;
		//ROS_INFO("Current x position is: %f", px);
		//ROS_INFO("Current y position is: %f", py);
	}
	
	void StageLaser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number
		
	}
	
	//custom resident callback function, you get the message object that was sent from Resident
	void residentStatusCallback(se306_project1::ResidentMsg msg)
	{
		// do something with the values
		// msg.robot_id = robot_id;
		// ResidentMsg.health = health;
		// ResidentMsg.boredom = boredom;
		// ResidentMsg.hunger = hunger;
		// ResidentMsg.x = px;
		// ResidentMsg.y = py;
		// ResidentMsg.theta = theta;
		// ResidentMsg.robot_type = "Resident";
		ROS_INFO("Resident hunger is: %d", msg.health);

	}

	/* 
	Model for all periodic events - start time, stop time, period. The issue with this at the moment is that
	any behaviour defined here will occur at the end of the duration provided when creating the timer. This
	could easily solved if this method had access to the value of that duration - I'll look into solving this
	later. It's more of a usabiity issue than a usefulness one, as you can  adjust the startTime and endTime
	but I'd prefer it to be more intuitive.
	NOT CURRENTLY SUPPORTED: Events on certain days (other than every x number of days), non-periodic events 
							 during the defined period (startTime - endTime)
	*/
	void medicationCallback(const ros::TimerEvent&) {
		int startTime = time_conversion::simHoursToRealSecs(6); // Start callback at 6am
		int endTime = time_conversion::simHoursToRealSecs(12); // Stop callback at 12pm

		int tnow = ros::Time::now().toSec(); // The simulation time now
		int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

		// Behaviour should only occur if the simulation time is between the specified start and end times.
		if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) { // Note that this will run at the end of the duration specified for the timer.
			ROS_INFO("providing medication"); 
			std::ostringstream s;
			s << tnow; 
			//ROS_INFO(s.str().c_str()); // Just shows elapsed seconds
		}
	}
	
	public:
	int run(int argc, char **argv)
	{
	
	 //initialize robot parameters
		//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
		theta = M_PI/2.0;
		px = 10;
		py = 20;
		
		//Initial velocity
		linear_x = 0.2;
		angular_z = 0.2;
		
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode1");
	
	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;
	
	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
	
	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Assistant::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Assistant::StageLaser_callback, this);
	
	//custom Resident subscriber to "resident/state"
	ros::Subscriber Resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Assistant::residentStatusCallback, this);
	
	ros::Rate loop_rate(10);
	
	//a count of howmany messages we have sent
	int count = 0;
	
	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	// Periodic callback
	int dur2 = time_conversion::simHoursToRealSecs(2); // Perform callback every 2 simulation hours
	ros::Timer medicationTimer = n.createTimer(ros::Duration(dur2), &Assistant::medicationCallback, this); 

	// // Testing getting parameters
	// std::string robotname;
 //   	n.getParam("robotname", robotname);
 //   	ROS_INFO("Name: ");
 //   	ROS_INFO(robotname.c_str());
	
	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
	        
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		ros::spinOnce();
	
		loop_rate.sleep();
		++count;
	}
	
	return 0;
	
	}
	
	//// Return type of robot
	//Type get_Type(){
	
	//}
	
	//// Get id of robot
	//int get_id(){
	
	//}
	
	//// Gives medication to the resident
	//void give_medication(){
		
	//}
	
	//// Cooks for the resident
	//void cook(){
	
	//}
	
	//// Entertain the resident
	//void entertain(){
	
	//}
	
	//// Gives companionship to resident
	//void give_companionship(){
	
	//}
};	

/* 
	Redirects to main function (run()) of the node.
*/
int main(int argc, char **argv) {
	Assistant *a = new Assistant;
	a->Assistant::run(argc, argv);
}