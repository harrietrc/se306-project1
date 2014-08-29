#include "ros/ros.h"
#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	protected:
		bool emergency = false; // required if timing/scheduling is done within this class
		bool finishedConvo = false;

		bool visitResident();
		bool doConverse();
		virtual void delegate(se306_project1::ResidentMsg msg) = 0;

	public:
		virtual int run(int argc, char *argv[]) = 0; 
};
