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
		int day;
		priorityQueue stateQueue; /*!< Queue of resident states */
		std::string residentState; /*!< The current resident state */
		bool hasShowered = false; /*!< True if the resident has showered */

	public:

		Resident() : Agent(){
			day = 1;
			health = 100;
			hunger = 0;
			boredom = 0;
			stateQueue = priorityQueue();
			residentState = stateQueue.checkCurrentState();
			originName = "ResidentOrigin";
			currentCheckpoint = std::make_pair(26,48);
			px = 26;
			py = 48;
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
		void wakeCallback(const ros::TimerEvent&);
		void sleepCallback(const ros::TimerEvent&);
		void caregiverServicesCallback(const ros::TimerEvent&);
		void caregiverServicesDoneCallback(const ros::TimerEvent&);
		void hungerCallback(const ros::TimerEvent&);
		void friendsCallback(const ros::TimerEvent&);
		void friendsDoneCallback(const ros::TimerEvent&);
		void caregiver_callback(const std_msgs::String::ConstPtr& msg);
		void checkStatus();


};
