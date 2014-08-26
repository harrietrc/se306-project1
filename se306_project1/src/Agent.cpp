#include "Agent.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/AssistantMsg.h"
#include <vector>

using namespace boost; // Useful for graphs
using namespace std;

/** Array of the names of checkpoints. Necessary for the initialisation of the checkpoints vector. */
const char* nameArr[] = { 
		"cp0","cp1", "cp2"
};

std::vector<std::string> checkpointNames(begin(nameArr), end(nameArr)); /*!< Vector of checkpoint names. See nameArr[]. */

/* -- Graph -- */

typedef property<vertex_name_t, std::string> VertexProperty; /*!<  Will allow us to retrieve vertex names from vertex references */
typedef adjacency_list <vecS, vecS, undirectedS, VertexProperty> vector_graph_t; /*!< Graph of checkpoint names */
const int checkpointNum = checkpointNames.size(); /*!< Number of checkpoints. */
vector_graph_t g(checkpointNum); // Our graph
std::map<std::string, vector_graph_t::vertex_descriptor> indices; /*!< Map that corresponds checkpoint names to the vertices in the graph. */

/* -- Edges -- */

typedef std::pair <std::string, std::string> E;
E paths[] = { E ("cp0", "cp1"), E ("cp1", "cp2")}; /*!< Defines edges between checkpoints */

/* -- Map of names to co-ordinates -- */

typedef std::string CheckpointName; // Key
typedef std::pair<int, int> Checkpoint; // Value
typedef std::map<CheckpointName, Checkpoint> CheckpointMap; /*!< Map with checkpoint names as keys and checkpoint co-ordinates as values */
CheckpointMap c;

/* -- Pathing -- */

int cc; /*!< The index of the agent's current checkpoint in the path. */
std::vector<std::pair<double, double> > path; /*!< The agent's path to a specified goal. */

/**
 *	@brief Finds the shortest path between 2 checkpoints, and returns the path as co-ordinates.
 *	Uses breadth first search - Boost recommends this over Dijkstra's algorithm for graphs with uniformly weighted edges.
 * 	Sets path member variable.
 *	@param startName The name of the start checkpoint as a string (e.g. 'kitchen')
 *	@param endName The name of the goal checkpoint as a string (e.g. 'bathroom')
 */
void Agent::getShortestPath(std::string startName, std::string endName) {

}

/**
 *	@brief Creates a graph of checkpoint names, provided the vector of names and the array of edges.
 */
void Agent::makeGraph() {

}

/**
 *	@brief Associates checkpoint names with checkpoint co-ordinates
 *	To be used in conjunction with a graph of checkpoint names, representing paths between checkpoints. Could be replaced
 *	by adding the co-ordinates to bundled properties in the property map of the graph.
 */
void Agent::checkpointMap() {

}

/* -- Stage callbacks -- */

/**
 *	@brief Updates the agent's x position, y position, and angle to reflect its current pose.
 *	@param msg Odometry message from odom topic
 */


void Agent::StageOdom_callback(nav_msgs::Odometry msg) {


	//Converting from quaternion to radians
	currentAngle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		currentAngle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}
	//Rounding to 3 decimal places
	currentAngle = ((int)(currentAngle * 1000 + .5) / 1000.0);


	//Update the current position
	px = msg.pose.pose.position.x;
	py = msg.pose.pose.position.y;

	if (isMoving == true){
		move();
	}
}

/*
 * MOVE METHODS {START}
 */



void Agent::move(){


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
			nextCheckpoint = shortestPath.at(shortestPathIndex);

			checkpointAngle = calculateGoalAngle(nextCheckpoint);
			//ROS_INFO("Goal Angle: %f", checkpointAngle);

			isClockwise = isTurnClockwise();
		}

	}

	if (!isFacingCorrectly){
		turn();
	}else{
		moveForward(nextCheckpoint);
	}

}



void Agent::turn(){
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



void Agent::moveForward(pair<double,double> nextCheckpoint){

	linear_x = 5;
	double minLinearX = 1.5;

	double distanceFromCheckpoint = sqrt(pow((nextCheckpoint.first - px),2) + pow((nextCheckpoint.second - py),2));

	// Check to ensure that linear velocity doesn't decrease if the distance between the checkpoints is higher than 40.
	double distanceRatio = (distanceFromCheckpoint / 40);
	if (distanceRatio > 1){
		distanceRatio = 1;
	}

	linear_x = minLinearX + linear_x * sin (distanceRatio * M_PI /2);

	if (distanceFromCheckpoint <= 0.5){
		currentCheckpoint = nextCheckpoint;
		isFacingCorrectly = false;
		linear_x = 0;
	}

}


bool Agent::isTurnClockwise(){

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
double Agent::calculateGoalAngle(pair<double, double> goalCheckpoint){

	//Finding the vector that the robot is facing and the goal vector
	double goalVectorX = goalCheckpoint.first - px;
	double goalVectorY = goalCheckpoint.second - py;

	double goalAngle = atan2(goalVectorY, goalVectorX); //pi <= goalAngle < -pi

	if (goalAngle < 0) {
		goalAngle = goalAngle * -1;
	} else if (goalAngle == 2 * M_PI) {
		goalAngle = 0;
	} else {
		goalAngle = 2 * M_PI - goalAngle;
	}

	//rounding goalAngle to three decimal places
	goalAngle = ((int)(goalAngle * 1000 + .5) / 1000.0);

	return goalAngle;
}

/*
 * MOVE METHODS {END}
 */

