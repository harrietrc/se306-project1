#include <se306_project1/src/Agent.h>
#include "std_msgs/String.h"

/**
*	@brief Class for robot assistants.
*/
class Assistant : public Agent
{

	protected:
		void delegate(se306_project1::ResidentMsg msg);
		bool doMedication(const ros::TimerEvent&);
		bool doCooking();
		bool doCleaning();
		bool doEntertainment();
		bool coordinate();

	public:
		int run(int argc, char *argv[]);

};
