#include "Visitor.h"

class Friend : public Visitor
{
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		
		// Converse with the resident
		//void converse()
};
