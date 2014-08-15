#include "Visitor.h"

/**
*	@brief Class for Nurse nodes.
*/
class Nurse : public Visitor
{
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		
		// restore health of the resident
		//void restore_health()
};
