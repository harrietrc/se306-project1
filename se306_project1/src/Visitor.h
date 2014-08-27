#include "ros/ros.h"
#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	private:
		bool emergency = false; // required if timing/scheduling is done within this class
		bool finishedConvo = false;

	protected:
		void visitResident();
		void doTimedVisit(const ros::TimerEvent&);
		void delegate(se306_project1::ResidentMsg msg);
		bool doConverse();

	public:
		int run(int argc, char *argv[]); 
};
