#include <nav_msgs/Odometry.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/AssistantMsg.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include "CheckPointGraph.hpp"

/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
*	Contains common navigation functionality and properties.
*/
class Agent
{
	public:
		Agent(){
			linear_x = 0;
			angular_z = 0;
			px = 0;
			py = 0;
			currentAngle = 0;
			// currentCheckpoint = std::make_pair(0,0); // Needs to be initialised in subclasses!!
			// shortestPath.push_back(currentCheckpoint); // Needs to be removed later!!
			isMoving = true;
			isFacingCorrectly = false;
			shortestPathIndex = 0;
			checkpointAngle = 0;
			isClockwise = true;
			robot_id = 0;
		}

		void StageOdom_callback(nav_msgs::Odometry msg);

	protected:
		// Sets origin name
		virtual void setOriginName();

		void setPath(std::string start, std::string end);

		// Checkpoint graph object
		CheckPointGraph g;

		// The name of this agent's origin checkpoint
		std::string originName = "Assistant1Origin"; //Temporary!

		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::string currentCheckpoint;

		//shortestPath
		std::vector <std::string> shortestPath;
		int shortestPathIndex;
		bool isFacingCorrectly;

		//moving status of the robot
		bool isMoving;

		int robot_id; /*!< Robot's ID */

		double checkpointAngle;
		bool isClockwise;

		void turn();
		void moveForward(std::string nextCheckpoint);
		double calculateGoalAngle(std::pair<double,double> goalCheckpoint);
		void move(std::string goal);
		bool isTurnClockwise();

		/* -- Communication and co-ordination -- */
	
};
