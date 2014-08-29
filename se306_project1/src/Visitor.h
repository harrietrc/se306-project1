#include "ros/ros.h"
#include "Agent.h"

/**
*	@brief Class for the implementation of visitors - i.e. Caregivers, Nurses, Doctors, Friends, and Relatives.
*/
class Visitor : public Agent {

	protected:
		bool emergency = false; /*!< True if the resident is ready to hospitalise (nurse must be adjacent to them). required if timing/scheduling is done within this class */
		bool finishedConvo = false; /*!< True if the visitor has finished conversing with the resident. */

		bool visitResident();
		bool doConverse();

	public:
		virtual int run(int argc, char *argv[]) = 0; 
		Visitor() : Agent(){

		}

};
