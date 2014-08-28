#include "Agent.h"
#include "priorityQueue.h"

/**
*	@brief Class representing the Resident.
*/
class Resident : public Agent
{
	private:
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */

		priorityQueue stateQueue;

		std::string residentState;

	public:

		Resident() : Agent(){
			health = 100;
			hunger = 0;
			boredom = 0;
			stateQueue = priorityQueue();
			residentState = stateQueue.checkCurrentState();
		}

		/**
		*	@brief Main function that controls agent events.
		*/
		int run(int argc, char *argv[]);

		void doctor_callback(se306_project1::DoctorMsg msg);
		void assistant_callback(se306_project1::AssistantMsg msg);
		void friend_callback(const std_msgs::String::ConstPtr& msg);
		bool doSleep(const ros::TimerEvent&);	
		void publishStatus(ros::Publisher Resident_state_pub);
		void triggerRandomEvents();
		void medicationCallback(const ros::TimerEvent&);


};
