#include "Agent.h"
#include "std_msgs/String.h"

class Assistant : public Agent
{

	protected:
		int health;
		int boredom;
		int hunger;
		
		//goal pose and orientation
		double goal_x;
		double goal_y;
		double px;
		double py;
		double goal_angle;

		bool running;
		bool isSet;

		//current pose and orientation of the robot
		double cur_angle;

		int cc; //current_checkpoint = 0;

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	

		int checkpoints[12][2] = {  
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
		{30, 25},
		{30,50}
		};

		bool cooking;


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
