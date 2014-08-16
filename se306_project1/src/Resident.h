#include "Agent.h"

/**
*	@brief Class representing the Resident.
*/
class Resident : public Agent
{
	protected:
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */
		
		//goal pose and orientation
		double goal_x; /*!< The x position of the agent's goal */
		double goal_y; /*!< The y position of the agent's goal */
		double px; /*!< The agent's initial x position */
		double py; /*!< The agent's initial y position */
		double goal_angle;  /*!< The angle that the agent must face to approach the goal, defined in reference to the co-ordinate system */

		//bool running;
		bool isSet; /*!< The angle that the agent must face to approach the goal, defined in reference to the co-ordinate system */

		//current pose and orientation of the robot
		double cur_angle; /*!< The agent's current facing in reference to the co-ordinate system */

		int cc; //current_checkpoint = 0;

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	/*!< linear_x and angular_z for the robot */
			


	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		void doctor_callback(se306_project1::DoctorMsg msg);
		double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py);
		std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
		void randomCheckpointCallback(const ros::TimerEvent&);
		void assistant_callback(se306_project1::AssistantMsg msg);
		std::pair<double, double>  movePath(int path[][2], int pathLength);
		void checkpointMap();
		void makeGraph();
		std::vector<std::pair<double, double> > shortestPath(std::string startName, std::string endName);

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
