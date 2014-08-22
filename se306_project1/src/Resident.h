#include "Agent.h"

/**
*	@brief Class representing the Resident.
*/
class Resident : public Agent
{
	private:
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */

	public:


		void turn();
		void moveForward(std::pair<double,double> nextCheckpoint);
		double calculateGoalAngle(std::pair<double, double> goalCheckpoint);
		void move();
		bool isTurnClockwise();
		std::pair<double, double>  movePath(int path[][2], int pathLength);

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
		// Return type of robot
		// MIGHT HAVE TO RETURN A STRING BECAUSE ROS DOESN'T SUPPORT ENUM IN MESSAGES
		//Type get_Type()
	
		// Get id of robot
		//int get_id(){
	
		// Wakes up
		//void wake_up()
	
		// Eat
		//void eat()
	
		// Takes medicine
		//void take_medicine()
	
		// Accepts entertainment
		//void accept_entertainment()
		
};
