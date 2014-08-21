#include "Visitor.h"

/**
*	@brief Class for Nurse nodes.
*/
class Nurse : public Visitor
{
	protected:
		bool hospitalise; /*!< Indicates whether the Doctor should take the resident to the  hospital. */

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char *argv[]);
		
		void delegate(se306_project1::ResidentMsg msg);
		bool doHospitalise();
};
