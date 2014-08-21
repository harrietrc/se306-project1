#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent
{

	public:
		//void StageOdom_callback(nav_msgs::Odometry msg);

		//int run(int argc, char *argv[]); //Doesnt work for some reason

		void delegate(const ros::TimerEvent&);

		bool meetResident(se306_project1::ResidentMsg msg);


	/*
	public:
			// Return type of robot
		Type get_Type(){
	
		}
	
		// Get id of robot
		int get_id(){
	
		}
	
		// Visit the resident
		void visit(){
		
		}
	*/
};
