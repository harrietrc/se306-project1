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

using namespace boost; // Useful for graphs

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
void Agent::shortestPath(std::string startName, std::string endName) {

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

}

/* -- Movement -- */

/**
*	@brief Sets the agent's linear and angular velocity based on its path.
*	Identifies the goal checkpoint based on the current checkpoint and uses this goal and the agent's pose to find the direction (and speed) to move in 
*	in order to reach the goal.
*	@returns The angular and linear velocity that the agent should adopt in order to reach the next checkpoint.
*/
std::pair<double, double> Agent::movePath() {
	return std::make_pair(0, 0); //stub
}

/**
*	@brief Keeps the agent moving by changing linear_x ad angular_z.
*	@return _ret linear and angular velocity for the agent. 
*/
std::pair<double, double> Agent::move() {
	return std::make_pair(0, 0); //stub
}

/**
*	@brief Given the agent's current angle, this function calculates the angle to the goal.
*/
double Agent::calc_goal_angle() {
	return 0;
}
