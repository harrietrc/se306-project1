#include "Visitor.h"

/**
*	@brief Class for Friend nodes.
*/
class Friend3 : public Visitor
{	
	protected:
		void delegate(se306_project1::ResidentMsg msg);
		void doTimedVisit(const ros::TimerEvent&);

	public:
		int run(int argc, char *argv[]);
		Friend3() {
			originName = "Friend3Origin";
		}
};