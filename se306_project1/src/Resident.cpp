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

//int a = 0;

using namespace std;
/**
 *	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
 *	@note Rounding is used to calculate the current angle. This approximation is accounted for by using threshholds when processing angles.
 *	@param msg Odometry message from odom topic
 */
void Resident::StageOdom_callback(nav_msgs::Odometry msg)
{
	//a++;
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

	if (currentCheckpoint.first == nextCheckpoint.first &&
			currentCheckpoint.second == nextCheckpoint.second){

		shortestPathIndex++;
		if (shortestPathIndex >= shortestPath.size()){
			shortestPathIndex = 0;
			isMoving = false;
			return;
		}else{
			checkpointAngle = calculateGoalAngle(shortestPath.at(shortestPathIndex));
			//ROS_INFO("Goal Angle: %f", checkpointAngle);

			isClockwise = isTurnClockwise();
			nextCheckpoint = shortestPath.at(shortestPathIndex);
		}

	}

	if (!isFacingCorrectly){
		turn();
	}else{
		moveForward(nextCheckpoint);
	}

}

void Resident::turn(){
	//	if (a % 10 == 0){
	//		ROS_INFO("Goal Angle: %f", checkpointAngle * 180 / M_PI);
	//		ROS_INFO("Current Angle: %f", currentAngle * 180 / M_PI);
	//		ROS_INFO("Angle Difference, %f", (currentAngle - checkpointAngle) * 180 / M_PI);
	//	}

	angular_z = 1.5;
	double minAngularZ = 0.05;

	if (isClockwise){
		angular_z = angular_z * -1;
		minAngularZ = minAngularZ * -1;
	}


	double angleDifference = fabs(checkpointAngle - currentAngle);

	if (angleDifference > M_PI) {
		angleDifference = 2 * M_PI - angleDifference;
	}

	angular_z = minAngularZ + angular_z * cos(M_PI/2 - angleDifference);

	if (fabs(angleDifference) <= 0.02){
		angular_z = 0.0049;
		if (isTurnClockwise()){
			angular_z = angular_z * -1;
		}
		if (fabs(angleDifference) <= 0.005){
			angular_z = 0;
			isFacingCorrectly = true;
		}
	}
}

void Resident::moveForward(pair<double,double> nextCheckpoint){

	linear_x = 5;
	double minLinearX = 1.5;

	double distanceFromCheckpoint = sqrt(pow((nextCheckpoint.first - px),2) + pow((nextCheckpoint.second - py),2));

	linear_x = minLinearX + linear_x * sin ((distanceFromCheckpoint / 40) * M_PI /2);

	ROS_INFO("Linear_x %f", linear_x);

	if (distanceFromCheckpoint <= 0.5){
		currentCheckpoint = nextCheckpoint;
		isFacingCorrectly = false;
		linear_x = 0;
	}

}

bool Resident::isTurnClockwise(){

	double angleDifference = checkpointAngle - currentAngle;
	if (angleDifference > 0){
		if (fabs(angleDifference) < M_PI){
			return true;
		}
	}else{
		if (fabs(angleDifference) > M_PI){
			return true;
		}
	}

	return false;
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
 *	@brief Main function for the Resident process.
 *	Controls node setup and periodic events.
 */
int Resident::run(int argc, char *argv[]) {

	currentCheckpoint = make_pair(30,25);
	pair<double, double> c1 = make_pair(40,30);
	pair<double, double> c2 = make_pair(30,40);

	shortestPath.push_back(currentCheckpoint);
	shortestPath.push_back(c1);
	shortestPath.push_back(c2);

	isMoving = true;
	isFacingCorrectly = false;
	shortestPathIndex = 0;
	checkpointAngle = 0;
	isClockwise = true;

	linear_x = 0;
	angular_z = 0;

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
