#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver : public Visitor
{
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		
		// Get id of robot
		//int get_id()

		// Helps resident with shower
		//void shower()
	
		// Feeds the resident
		//void feed()
	
		// Helps resident with exercise
		//void exercise()
	
		// Have a conversation with the resident
		//void converse()
	
		// Receives moral support
		//void moral_support()
};
