#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/AssistantMsg.h"

#include <sstream>
#include "math.h"
#include "time_conversion.hpp"
#include "Assistant.h"

//velocity of the robot
double linear_x;
double angular_z;

//goal pose and orientation
double goal_x;
double goal_y;
double goal_angle;
bool isSet = false;
int foodIsDelivered = 0;
bool moveToPoint = false;

//current pose and orientation of the robot
double px;
double py;
double cur_angle;

int cc = 1; //current_checkpoint = 0;
int foodIsReady = 0;
std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py); 
void StageOdom_callback(nav_msgs::Odometry msg); 

/**
*	@brief Updates the Assistant's x position, y position, and angle to reflect its current pose.
*	@note Rounding is used to calculate the current angle. This approximation is accounted for by using threshholds when processing angles.
*	@param msg Odometry message from odom topic
*/
void Assistant::StageOdom_callback(nav_msgs::Odometry msg)
{
	//Converting from quaternion to radians
	cur_angle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		cur_angle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}

	//Rounding to 3 decimal places
	cur_angle = ((int)(cur_angle * 1000 + .5) / 1000.0);
	
	
	//Update the current position
	px = msg.pose.pose.position.x + checkpoints[0][0];
	py = msg.pose.pose.position.y + checkpoints[0][1];

	
}

/**
*	@brief Causes the robot to move until the goal is reached.
*	When the goal is reached, the next checkpoint becomes the goal or if the list of checkpoints is exhausted, the robot returns
*	to its initial position (the first checkpoint)
*	@param path[][2] An array of checkpoints that forms a path.
*	@param pathLength The number of checkpoints in the path (-1, as counting starts at 0)
*	@return linear_x and angular_z
*/
std::pair<double, double> Assistant::movePath(int path[][2], int pathLength) {	
	
	std::pair<double, double> ret;	
	ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	
	//When goal reached
	if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5) && (py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {
	isSet = false;
		if (cc == pathLength) { //If at last checkpoint
			linear_x = 0;
		} else {
			cc++; //Increment checkpoint index
		}
		if (cc == pathLength) {
			linear_x = 0;
			goal_x = path[cc-1][0];
			goal_y = path[cc-1][1];
		} else {
			goal_x = path[cc][0];
			goal_y = path[cc][1];
		}
	
		//Account for delay by subtracting delay values from current pose and orientation
		goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle - M_PI/20, px - 0.1, py - 0.1); 
		//goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle, px, py);

	} else { //Do this until goal is reached
		ret = move(goal_x, goal_y, cur_angle, goal_angle, px, py);	
	}
	
	return ret;
}


/**
*	@brief Keeps the robot moving by changing linear_x ad angular_z.
*	@param goal_x The x position of the robot's goal
*	@param goal_y The y position of the robot's goal
* 	@param cur_angle The robot's current facing, in reference to the co-ordinate system.
*	@param goal_angle The angle that the robot must face in order to reach the goal.
*	@param px Initial x position
*	@param py Initial y position
*	@return _ret linear_x and angular_z
*/
std::pair<double, double> Assistant::move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py) 
{		
	
	std::pair<double,double>_ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	double moveSpeed = M_PI/2;
	moveSpeed = ((int)(moveSpeed * 1000 + .5) / 1000.0);

	//When the robot is facing the correct direction, start moving
	double threshold = cur_angle;//cur_angle-moveSpeed/10;
	//threshold = ((int)(threshold * 1000 + .5) / 1000.0);
//threshold = ((int)(threshold * 1000 + .5) / 1000.0);

	if ((goal_angle  == threshold) || isSet) {
		_ret.first = 5; //linear_x
		_ret.second = 0; //angular_z
		isSet = true;
	} else if ((goal_angle <= cur_angle + 0.75) && (goal_angle >= cur_angle - 0.75) )  {
		_ret.first = 0; //linear_x
		_ret.second = fabs(cur_angle - goal_angle);//0.001; //angular_z
		if (goal_angle == cur_angle) {
			isSet = true;		
		}

	} else {
		_ret.first = 0; //linear_x
		_ret.second = moveSpeed; //angular_z
		isSet = false;
	}


	if ((px-0.1 <= goal_x + 0.5) && (px-0.1 >= goal_x - 0.5) && (py-0.1 <= goal_y + 0.5) && (py-0.1 >= goal_y - 0.5)) {	
			_ret.first = 0; 
			isSet = false;
	}

	return _ret; 
}

/**
*	@brief Given the robot's current angle, this function calculates the angle to the goal.
*	cur_angle, goal_x, goal_y, px, and py are class fields but are also passed as parameters.
*	@param goal_x The x co-ordinate of the goal
*	@param goal_y The y co-ordinate of the goal
*	@param cur_angle The robot's current angle, in reference to the co-ordinate system
*	@param px The robot's initial x position
*	@param py The robot's initial y position
*	@param goal_angle The angle that the robot must rotate to face the goal, in reference to the co-ordinate system.
*/
double Assistant::calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py) 
{

	//Initial and goal vectors used to calculate goal theta
	double init_vector_x;
	double init_vector_y;
	double goal_vector_x;
	double goal_vector_y;
	double goal_angle;
	
	//Finding the vector that the robot is facing and the goal vector
	init_vector_x = cos(cur_angle);
	init_vector_y = sin(cur_angle);
	goal_vector_x = goal_x - px;
	goal_vector_y = goal_y - py;
	
	goal_angle = atan2(goal_vector_y, goal_vector_x); //pi <= goal_angle < -pi
	if (goal_angle < 0) {
		goal_angle = 2 * M_PI + goal_angle; //Remove sign, then add to pi
	} else if (goal_angle == 2 * M_PI) { //New goal angle =>   >0 to 6.283
		goal_angle = 0;
	}
	goal_angle = (2* M_PI) - goal_angle;

	//rounding goal_angle to three decimal places
	goal_angle = ((int)(goal_angle * 1000 + .5) / 1000.0);
	
	return goal_angle;
}

/**
*	@brief Callback function to process laser scan messsages.
*	You can access the range data from msg.ranges[i]. i = sample number
*	@note Currently blank as it is not in use. Navigation operates through a checkpoint system.
*	@param msg Single scan from a planar laser range finder
*/
void Assistant::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
}

/**
*	@brief Callback function that unpacks and processes resident status messages.
*	Assistant should subscribe to the ResidentMsg topic in order for this callback to be called. ResidentMsg is published by the Resident.
*	@note Currently this callback processes only resident hunger, controlling the cooking behaviour. More behaviours 
*	can be implemented later.
*	@param msg A custom ResidentMsg message that contains information about the resident's current status.
*/
void Assistant::residentStatusCallback(se306_project1::ResidentMsg msg)
{
	// do something with the values
	// msg.robot_id = robot_id;
	// ResidentMsg.health = health;
	// ResidentMsg.boredom = boredom;
	// ResidentMsg.hunger = hunger;
	// ResidentMsg.x = px;
	// ResidentMsg.y = py;
	// ResidentMsg.theta = theta;

	std::pair<double, double> velocityValues;	
	velocityValues = std::make_pair(0, 0);
	
	
	if ((sqrt(pow((msg.x - px),2) + pow((msg.y - py),2)) < 2.5) && foodIsDelivered == 0){
		linear_x = 0;
		angular_z = 0;
		moveToPoint = true;
		goal_x = px;
		goal_y = py;
		foodIsReady = 1;
	} 
	
	if(msg.hunger < 90 && cooking ==false)
	{
		cooking = true;
		velocityValues = movePath(checkpoints, 	11);
		linear_x = velocityValues.first;
		angular_z = velocityValues.second;
	} else if (msg.hunger >= 60)
	{
		cooking = false;
		
	}
	
	if (cooking || moveToPoint) {
		velocityValues = movePath(checkpoints, 	11);
		linear_x = velocityValues.first;
		angular_z = velocityValues.second;
	}

}

/**
*	@brief Periodic callback for the provision of medication that should act as a model for all periodic events.
*	Called by the ros::Timer in the run() function. Can specify start time, end time, and period.
*	@note The callback is called at the end of the duration specified for the timer.
*	@remark This could be adjustd in the future to allow for events that occur only on certain days - currently it accommodates
*	only for events that occur every x days (or multiple times per day).
*	@param TimerEvent& TimerEvent generated by a ros::Timer.
*/
void Assistant::medicationCallback(const ros::TimerEvent&) {
	int startTime = time_conversion::simHoursToRealSecs(6); // Start callback at 6am
	int endTime = time_conversion::simHoursToRealSecs(12); // Stop callback at 12pm

	int tnow = ros::Time::now().toSec(); // The simulation time now
	int dlen = time_conversion::getDayLength(); // The length of a simulation day, in seconds

	// Behaviour should only occur if the simulation time is between the specified start and end times.
	if (((tnow % dlen) > startTime) && ((tnow % dlen) < endTime)) { // Note that this will run at the end of the duration specified for the timer.
		//#DEBUG ROS_INFO("providing medication");
		std::ostringstream s;
		s << tnow; 
		//ROS_INFO(s.str().c_str()); // Just shows elapsed seconds
	}
}

/**
*	@brief Main function for the Assistant process.
*	Controls node setup and periodic events.
*/
int Assistant::run(int argc, char **argv)
{

	//Initial pose. This is the same as the pose used in the world file.
	px = checkpoints[cc-1][0];
	py = checkpoints[cc-1][1];
	cur_angle = 0;

	//Set goal pose 
	goal_x = checkpoints[cc][0];
	goal_y = checkpoints[cc][1];

	goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle, px, py);

	//Initial velocities
	linear_x = 0;
	angular_z = 0;

	//initialize robot parameters
	cooking = false;
		
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Assistant");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	ros::Publisher assistant_pub = n.advertise<se306_project1::AssistantMsg>("assistantStatus",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Assistant::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Assistant::StageLaser_callback, this);

	//custom Resident subscriber to "resident/state"
	ros::Subscriber Resident_sub = n.subscribe<se306_project1::ResidentMsg>("residentStatus",1000,&Assistant::residentStatusCallback, this);

	ros::Rate loop_rate(1000);

	//a count of howmany messages we have sent
	int count = 0;
	int i = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	
	se306_project1::AssistantMsg aMsg;

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
		
		//se306_project1::AssistantMsg msg;
		//msg.cooking = cooking;
	
		//status = "Cooking";
	/*	if (cooking == true && count % 10 == 0){
			i += 1;
			if (i == 30)
			{
				assistant_pub.publish(msg);
<<<<<<< HEAD
=======
				//ROS_INFO("Food is ready");
>>>>>>> 6f4325dfcc39ce8764a381daccd2cd95122ba38d
				i = 0;
			}
		}
		*/
		if (foodIsReady == 1) {
			aMsg.FoodDelivered = foodIsReady;
			foodIsReady = 0;
			assistant_pub.publish(aMsg);
			foodIsDelivered = 1;
		}
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}

/**
*	@brief Redirects to main function (run()) of the node.
*/
int main(int argc, char **argv) {
	Assistant *a = new Assistant();
	a->Assistant::run(argc, argv);
}
