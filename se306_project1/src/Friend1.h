#include "Visitor.h"

/**
*	@brief Class for Friend nodes.
*/
class Friend1 : public Visitor
{
	private:
		
	protected:
		void delegate(se306_project1::ResidentMsg msg);
		void doTimedVisit(const ros::TimerEvent&);

	public:
		int run(int argc, char *argv[]);
		void friendsDoneCallback(const ros::TimerEvent&);

};
