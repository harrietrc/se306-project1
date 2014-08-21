#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"

#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "Resident.h"	

using namespace std;
/**
 *	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
 *	@note Rounding is used to calculate the current angle. This approximation is accounted for by using threshholds when processing angles.
 *	@param msg Odometry message from odom topic
 */
void Resident::StageOdom_callback(nav_msgs::Odometry msg)
{
	//Converting from quaternion to radians
	currentAngle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		currentAngle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}

	//Rounding to 3 decimal places
	currentAngle = ((int)(currentAngle * 1000 + .5) / 1000.0);

//	currentAngle = currentAngle * (180 / M_PI);

//	if (count % 10 == 0){
//		ROS_INFO ("px %f", msg.pose.pose.position.x);
//		ROS_INFO ("py %f", msg.pose.pose.position.y);
//		ROS_INFO ("angle %f", currentAngle);
//	}

	px = msg.pose.pose.position.x;
	py = msg.pose.pose.position.y;

	if (isMoving == true){
		move();
	}


}

void Resident::move(){


	if (isMoving == false){
		isMoving = true;
		currentCheckpoint.first = 30;
		currentCheckpoint.second = 25;
		//Get the path stuff
	}

	pair<double, double> nextCheckpoint = shortestPath.at(shortestPathIndex);

	if (!isFacingCorrectly){
		turn(currentCheckpoint, nextCheckpoint);
	}else{
		moveForward(currentCheckpoint, nextCheckpoint);
	}

	if (currentCheckpoint.first == nextCheckpoint.first &&
		currentCheckpoint.second == nextCheckpoint.second){

		shortestPathIndex++;
		if (shortestPathIndex > shortestPath.size()){
			shortestPathIndex = 1;
			isMoving = false;
		}

	}

}

void Resident::turn(pair<double,double> currentCheckpoint, pair<double,double> nextCheckpoint){

}

void Resident::moveForward(pair<double,double> currentcheckpoint, pair<double,double> nextCheckpoint){

}

/**
 *	This function calculates the angle to the goal.
 */
double Resident::calculateGoalAngle(pair<double, double> goalCheckpoint){

	//Finding the vector that the robot is facing and the goal vector
	double goalVectorX = goalCheckpoint.first - px;
	double goalVectorY = goalCheckpoint.second - py;

	double goalAngle = atan2(goalVectorY, goalVectorX); //pi <= goalAngle < -pi

	if (goalAngle < 0) {
		goalAngle = goalAngle * -1;
	} else if (goalAngle == 2 * M_PI) { //New goal angle =>   >0 to 6.283
		goalAngle = 0;
	} else {
		goalAngle = 2 * M_PI - goalAngle;
	}

	//rounding goalAngle to three decimal places
	goalAngle = ((int)(goalAngle * 1000 + .5) / 1000.0);

	return goalAngle;
}

/**
 *	@brief Causes the agent to move until the goal is reached.
 *	When the goal is reached, the next checkpoint becomes the goal or if the list of checkpoints is exhausted, the agent returns
 *	to its initial position (the first checkpoint)
 *	@param path[][2] An array of checkpoints that forms a path.
 *	@param pathLength The number of checkpoints in the path (-1, as counting starts at 0)
 *	@return linear_x and angular_z
 */
//std::pair<double, double> Resident::movePath(int path[][2], int pathLength) {
//	std::pair<double, double> ret;
//	ret = std::make_pair(0, 0); //initialize pair. Used to get return.
//	//When goal reached
//
//	if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5) && (py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {
//		isSet = false;
//		if (cc == pathLength) { //If at last checkpoint
//			linear_x = 0;
//		} else {
//			cc++; //Increment checkpoint index
//		}
//		if (cc == pathLength) {
//			linear_x = 0;
//			goal_x = path[cc-1][0];
//			goal_y = path[cc-1][1];
//		} else {
//			goal_x = path[cc][0];
//			goal_y = path[cc][1];
//		}
//		//Account for delay by subtracting delay values from current pose and orientation
//		goalAngle = calc_goalAngle(goal_x, goal_y, currentAngle - M_PI/20, px - 0.1, py - 0.1);
//		//goalAngle = calc_goalAngle(goal_x, goal_y, currentAngle, px, py);
//
//	} else { //Do this until goal is reached
//		ret = move(goal_x, goal_y, currentAngle, goalAngle, px, py);
//	}
//	return ret;
//}





/**
 *	@brief Callback function to process laser scan messsages.
 *	You can access the range data from msg.ranges[i]. i = sample number
 *	@note Currently blank as it is not in use. Navigation operates through a checkpoint system.
 *	@param msg Single scan from a planar laser range finder
 */
void Resident::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number

}

/**
 *	@brief Keeps the agent moving by changing linear_x ad angular_z.
 *	@param goal_x The x position of the robot's goal
 *	@param goal_y The y position of the robot's goal
 * 	@param currentAngle The agent's current facing, in reference to the co-ordinate system.
 *	@param goalAngle The angle that the agent must face in order to reach the goal.
 *	@param px Initial x position
 *	@param py Initial y position
 *	@return _ret linear_x and angular_z
 */
//std::pair<double, double> Resident::move(double goal_x, double goal_y, double currentAngle, double goalAngle, double px, double py)
//{
//	std::pair<double,double>_ret = std::make_pair(0, 0); //initialize pair. Used to get return.
//	double moveSpeed = M_PI/2;
//	moveSpeed = ((int)(moveSpeed * 1000 + .5) / 1000.0);
//
//	//When the robot is facing the correct direction, start moving
//	double threshold = currentAngle;//-moveSpeed/10;
//	//threshold = ((int)(threshold * 1000 + .5) / 1000.0);
//
//	if ((goalAngle  == threshold) || isSet) {
//		_ret.first = 5; //linear_x
//		_ret.second = 0; //angular_z
//		isSet = true;
//	} else if ((goalAngle <= currentAngle + 0.6) && (goalAngle >= currentAngle - 0.6) )  {
//		_ret.first = 0; //linear_x
//		_ret.second = fabs(goalAngle - currentAngle); //angular_z
//		if (goalAngle == currentAngle) {
//			isSet = true;
//		}
//
//	} else {
//		_ret.first = 0; //linear_x
//		_ret.second = moveSpeed; //angular_z
//		isSet = false;
//	}
//
//
//	if ((px-0.1 <= goal_x + 0.5) && (px-0.1 >= goal_x - 0.5) && (py-0.1 <= goal_y + 0.5) && (py-0.1 >= goal_y - 0.5)) {
//		_ret.first = 0;
//		isSet = false;
//	}
//
//	return _ret;
//}


/**
 *	@brief Main function for the Resident process.
 *	Controls node setup and periodic events.
 */
int Resident::run(int argc, char *argv[]) {

	pair<double, double> c1 = make_pair(35,30);
	pair<double, double> c2 = make_pair(35,25);

	shortestPath.push_back(currentCheckpoint);
	shortestPath.push_back(c1);
	shortestPath.push_back(c2);

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");
	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);


	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Resident::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Resident::StageLaser_callback, this);

	//ros::Rate loop_rate(10);
	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		//ROS_INFO("Hunger %d",hunger);
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);


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
	Resident *a = new Resident();
	a->Resident::run(argc, argv);
}
