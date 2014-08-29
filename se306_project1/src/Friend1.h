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

		Friend1() : Visitor(){
			currentCheckpoint = std::make_pair(-23, -46);
			px = -23;
			py= -46;
		}
		int run(int argc, char *argv[]);
		void friendsDoneCallback(const ros::TimerEvent&);
};
