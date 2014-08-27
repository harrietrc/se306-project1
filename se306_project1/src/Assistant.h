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
		

	private:
		// functions
		void medicate(const ros::TimerEvent&);
		void cook();
		void clean();
		void entertain();
		void delegate(se306_project1::ResidentMsg msg);

		// variables
		bool atKitchen = false;
		bool finishedCooking = false;

		bool atBedroom = false;
		bool residentEntertained = false;

};
