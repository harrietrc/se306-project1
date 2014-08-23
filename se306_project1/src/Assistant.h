#include "Agent.h"

/**
*	@brief Class representing the Assistant.
*/
class Assistant : public Agent
{
	public:

		Assistant() : Agent(){
		}

		/**
		*	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
		*	@param msg Odometry message from odom topic
		*/
		void StageOdom_callback(nav_msgs::Odometry msg);

		/**
		*	@brief Virtual callback function to process laser scan messsages.
		*	@param msg Single scan from a planar laser range finder
		*/
		void StageLaser_callback(sensor_msgs::LaserScan msg);

		/**
		*	@brief Main function that controls agent events.
		*/
		int run(int argc, char *argv[]);
		
};
