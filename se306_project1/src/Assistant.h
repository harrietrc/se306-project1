#include "Agent.h"
#include "std_msgs/String.h"

/**
*	@brief Class for robot assistants.
*/
class Assistant : public Agent
{

	protected:
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */
		
		//goal pose and orientation
		double goal_x; /*!< The x position of the robot's goal */
		double goal_y; /*!< The y position of the robot's goal */
		double px; /*!< The robot's initial x position */
		double py; /*!< The robot's initial y position */
		double goal_angle; /*!< The angle that the robot must face to approach the goal, defined in reference to the co-ordinate system */

		bool running; 
		bool isSet; /*!< Indicates whether the robot is currently on its way to a goal. */

		//current pose and orientation of the robot
		double cur_angle; /*!< The robot's current facing in reference to the co-ordinate system */

		int cc; //current_checkpoint = 0;

		bool is_called; 

		std::pair<double,bool> goal_pair; 
		std::pair<double, double> ret;	/*!< linear_x and angular_z for the robot */

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

		bool cooking; /**< True if the robot is cooking */


	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);

		
		double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py);
		std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
		void randomCheckpointCallback(const ros::TimerEvent&);
		std::pair<double, double>  movePath(int path[][2], int pathLength);
		void residentStatusCallback(se306_project1::ResidentMsg msg);
		void medicationCallback(const ros::TimerEvent&);

		//// Return type of robot
		//Type get_Type()
	
		//// Get id of robot
		//int get_id()
	
		//// Gives medication to the resident
		//void give_medication()
	
		//// Cooks for the resident
		//void cook()
	
		//// Entertain the resident
		//void entertain()
	
		//// Gives companionship to resident
		//void give_companionship()
};
