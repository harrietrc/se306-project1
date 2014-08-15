#include "Visitor.h"
#include "std_msgs/String.h"

class Doctor : public Visitor
{
	protected:
		bool healResident;
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

		//current pose and orientation of the robot
		double cur_angle;

		int cc = 1; //current_checkpoint = 0;

		int checkpoints[4][2] = {
			{10, -7},
			{10, 1},
			{30, 20},
			{30, 25}
			};

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char *argv[]);


		double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py);
		std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
		void randomCheckpointCallback(const ros::TimerEvent&);
		std::pair<double, double>  movePath(int path[][2], int pathLength);
		void residentStatusCallback(se306_project1::ResidentMsg msg);
		void medicationCallback(const ros::TimerEvent&);

		// Restores health of the resident
		//void restore_Health()
		
};
