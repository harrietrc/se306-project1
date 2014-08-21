#include "Visitor.h"
#include "std_msgs/String.h"

/**
*	@brief Class for the Doctor nodes.
*/
class Doctor : public Visitor
{
	protected:
		bool healResident; /*!< Indicates whether the Doctor should heal the resident. */
		bool hospitalise; /*!< Indicates whether the Doctor should take the resident to the  hospital. */
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */
		
		//goal pose and orientation
		double goal_x; /*!< The x position of the agent's goal */
		double goal_y; /*!< The y position of the agent's goal */
		double px; /*!< The agent's initial x position */
		double py; /*!< The agent's initial y position */
		double goal_angle;  /*!< The angle that the agent must face to approach the goal, defined in reference to the co-ordinate system */

		bool running;

		//current pose and orientation of the robot
		double cur_angle;  /*!< The agent's current facing in reference to the co-ordinate system */

		int cc = 1; /*!< The index of the checkpoint that the agent is currently aiming for */
		//current_checkpoint = 0;

		/**
		*	@brief Set of checkpoints that the nodes can move to in the map.
		* 	Gives x position and y position for each co-ordinate
		*/
		int checkpoints[4][2] = {
			{10, -7},
			{10, 1},
			{30, 20},
			{30, 25}
			};

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	/*!< linear_x and angular_z for the robot */

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char *argv[]);


		double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py);
		std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
		void randomCheckpointCallback(const ros::TimerEvent&);
		std::pair<double, double>  movePath(int path[][2], int pathLength);
		void delegate(se306_project1::ResidentMsg msg);
		void medicationCallback(const ros::TimerEvent&);

		bool doHeal(se306_project1::ResidentMsg msg);
		bool doHospitalise();
		
};
