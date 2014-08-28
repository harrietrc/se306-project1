#include "std_msgs/String.h"
#include "Agent.h"

/**
*	@brief Class representing the Assistant.
*/
class Assistant : public Agent
{
	public:

		Assistant() : Agent(){

			atKitchen = false;
			finishedCooking = false;

			atBedroom = false;
			residentEntertained = false;

			isMedicated = false;
		}

		/**
		*	@brief Main function that controls agent events.
		*/
		int run(int argc, char *argv[]);
		

	private:
		// functions
		void medicate(se306_project1::ResidentMsg msg);
		void cook(se306_project1::ResidentMsg msg);
		void clean();
		void entertain(se306_project1::ResidentMsg msg);
		void delegate(se306_project1::ResidentMsg msg);

		ros::Publisher Assistant_state_pub;

		// Boolean variables
		bool atKitchen;
		bool finishedCooking;

		bool atBedroom;
		bool residentEntertained;

		int entertainmentCounter = 0;

		bool isMedicated;
};
