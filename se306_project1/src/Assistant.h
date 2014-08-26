#include "std_msgs/String.h"
#include "Agent.h"

/**
*	@brief Class representing the Assistant.
*/
class Assistant : public Agent
{
	public:

		Assistant() : Agent(){
		}

		/**
		*	@brief Main function that controls agent events.
		*/
		int run(int argc, char *argv[]);
		

	protected:
		bool doMedication(const ros::TimerEvent&);
		bool doCooking();
		bool doCleaning();
		bool doEntertainment();
		bool coordinate();

};
