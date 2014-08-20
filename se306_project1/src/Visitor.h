#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	protected:
		void visitResident();
		void doTimedVisit(const ros::TimerEvent&, int startTime, int endTime);
		bool doConverse();

	public:

};
