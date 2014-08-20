#include "Agent.h"

/**
*	@brief Class representing the Resident.
*/
class Resident : public Agent
{
	protected:
		int health; /*!< Resident health */
		int boredom; /*!< Resident boredom */
		int hunger; /*!< Resident hunger */	
		bool doSleep(const ros::TimerEvent&);	

	public:
		int run(int argc, char **argv);
		void doctor_callback(se306_project1::DoctorMsg msg);
		void assistant_callback(se306_project1::AssistantMsg msg);
		
};
