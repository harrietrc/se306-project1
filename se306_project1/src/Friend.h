#include "Visitor.h"

/**
*	@brief Class for Friend nodes.
*/
class Friend : public Visitor
{
	private:
		
	protected:
		void delegate(se306_project1::ResidentMsg msg);
		void doTimedVisit(const ros::TimerEvent&);

	public:

		Friend() : Visitor(){
			currentCheckpoint = std::make_pair(-20, -46);
			px = -20;
			py= -46;
		}
		int run(int argc, char *argv[]);
		void friendsDoneCallback(const ros::TimerEvent&);

};
