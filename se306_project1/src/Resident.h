#include "Agent.h"

/**
*	@brief Class representing the Resident.
*/
class Resident : public Agent
{
	protected:
		bool doSleep(const ros::TimerEvent&);	
		void publishStatus();

	public:
		int run(int argc, char **argv);
		void doctor_callback(se306_project1::DoctorMsg msg);
		void assistant_callback(se306_project1::AssistantMsg msg);
		
};
