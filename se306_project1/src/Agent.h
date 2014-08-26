#include <nav_msgs/Odometry.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/AssistantMsg.h"
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

/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
*	Contains common navigation functionality and properties.
*/
#include <stdlib.h>
#include <sstream>
#include <vector>


class Agent
{
	public:
		Agent(){
			linear_x = 0;
			angular_z = 0;
			px = 0;
			py = 0;
			currentAngle = 0;
			currentCheckpoint = std::make_pair(0,0); // Needs to be initialised in subclasses!!
			shortestPath.push_back(currentCheckpoint); // Needs to be removed later!!
			isMoving = true;
			isFacingCorrectly = false;
			shortestPathIndex = 0;
			checkpointAngle = 0;
			isClockwise = true;
			robot_id = 0;

		}

		void StageOdom_callback(nav_msgs::Odometry msg);

	protected:
		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::pair<double, double> currentCheckpoint;

		//shortestPath
		std::vector <std::pair<double,double> > shortestPath;
		int shortestPathIndex;
		bool isFacingCorrectly;

		//moving status of the robot
		bool isMoving;
		/**
		*	@brief Set of checkpoints that the nodes can move to in the map.
		* 	Gives x position and y position for each co-ordinate
		*/
		int checkpoints[11][2] = {  
		{30, 25},
		{30, 7}, 
		{40, 7},
		{40, 8},
		{38,8},
		{38,7},
		{40,7},
		{30,7},
		{30, 25},
		{34,20},
		{30, 25}
		};

		int robot_id; /*!< Robot's ID */

		double checkpointAngle;
		bool isClockwise;

		void makeGraph();
		void checkpointMap();
		void turn();
		void moveForward(std::pair<double,double> nextCheckpoint);
		double calculateGoalAngle(std::pair<double,double> goalCheckpoint);
		void move();
		bool isTurnClockwise();

		/* -- Communication and co-ordination -- */

		void delegate(se306_project1::ResidentMsg msg) {} /*!< Callback that calls callbacks */

		void getShortestPath(std::string startName, std::string endName);
	
};
