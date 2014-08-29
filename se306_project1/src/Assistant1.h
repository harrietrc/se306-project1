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

		ros::Publisher Assistant_state_pub; /*!< Publishes to resident, keeping them up to date as to status of behaviours. */
		ros::Publisher Assistant_comm_pub; /*!<  */
		// Boolean variables
		bool atKitchen; /*!< True if the assistant is at the kitchen */
		bool finishedCooking; /*!< True if the assistant has finished cooking */
		bool foodDelivered; /*!< True if food has been delivered to the resident */
		bool atBedroom; /*!< True if the assistant is at the bedroom */
		bool residentEntertained; /*!< True if the resident has been entertained */

		int entertainmentCounter = 0;

		bool isMedicated; /*!< True if the resident has been medicated */
};
