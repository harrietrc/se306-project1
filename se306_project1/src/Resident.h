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
		
		//goal pose and orientation
		double goal_x; /*!< The x position of the agent's goal */
		double goal_y; /*!< The y position of the agent's goal */
		double goal_angle;  /*!< The angle that the agent must face to approach the goal, defined in reference to the co-ordinate system */

		//bool running;
		bool isSet; /*!< The angle that the agent must face to approach the goal, defined in reference to the co-ordinate system */

		int cc; //current_checkpoint = 0;

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	/*!< linear_x and angular_z for the robot */
			


	public:

		void turn(std::pair<double, double> currentCheckpoint, std::pair<double,double> nextCheckpoint);
		void moveForward(std::pair<double, double> currentCheckpoint, std::pair<double,double> nextCheckpoint);

		double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py);
		void move();
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
