#include "Visitor.h"

class Doctor : public Visitor
{
	protected:
		bool healResident;

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);

		void residentStatusCallback(se306_project1::ResidentMsg msg);

		// Restores health of the resident
		//void restore_Health()
		
};
