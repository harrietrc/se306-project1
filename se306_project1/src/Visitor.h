#include "ros/ros.h"
#include <se306_project1/src/Agent.h>

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	protected:
		void visitResident();
		void doTimedVisit(const ros::TimerEvent&);
		bool doConverse();

	public:

};