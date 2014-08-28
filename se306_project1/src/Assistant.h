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
			foodDelivered = false;

			isMedicated = false;
		}

		/**
		*	@brief Main function that controls agent events.
		*/
		int run(int argc, char *argv[]);
		

	private:
		// functions
		void medicate();
		void cook();
		void clean();
		void entertain();
		void delegate(se306_project1::ResidentMsg msg);
	//	void publishStatus()

		// Boolean variables
		bool atKitchen;
		bool finishedCooking;
		bool foodDelivered;

		bool atBedroom;
		bool residentEntertained;

		int entertainmenCounter = 0;

		bool isMedicated;
};
