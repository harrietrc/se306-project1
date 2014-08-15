#include "Visitor.h"

/**
*	@brief Class for Relative nodes.
*/
class Relative : public Visitor
{
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		
		// Converse with the resident
		//void converse()
};
