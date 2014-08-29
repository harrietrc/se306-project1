#include "std_msgs/String.h"
#include "Agent.h"

/**
*	@brief Class representing the Assistant.
*/
class Assistant1 : public Agent
{
	public:

		Assistant1() : Agent(){

			currentCheckpoint.first = 30;
			currentCheckpoint.second = -6;
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
		void clean(se306_project1::ResidentMsg msg);

		ros::Publisher Assistant_state_pub;
		ros::Publisher Assistant_comm_pub;
		// Boolean variables
		bool atKitchen;
		bool finishedCooking;
		bool foodDelivered;
		bool atBedroom;
		bool residentEntertained;

		int entertainmentCounter = 0;

		bool isMedicated;
};
