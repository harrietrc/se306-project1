#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver : public Visitor
{
	protected:
		bool needs_food;
		bool needs_excercise;
		bool needs_shower;
		bool needs_moral_support;

		
	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char *argv[]);

		void delegate(const ros::TimerEvent&);

		bool doEatSupport(se306_project1::ResidentMsg msg);
		bool doMoralSupport(se306_project1::ResidentMsg msg);
		bool doShowerSupport(se306_project1::ResidentMsg msg);
		bool doExcerciseSupport(se306_project1::ResidentMsg msg);
		
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
