#include "Agent.h"

class Assistant : public Agent
{
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);

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
