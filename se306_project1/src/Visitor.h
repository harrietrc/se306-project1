#include "ros/ros.h"
#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	protected:
		void visitResident();
		void doTimedVisit(const ros::TimerEvent&);
		bool doConverse();
		virtual int run(int argc, char *argv[]) = 0; 

	public:

};