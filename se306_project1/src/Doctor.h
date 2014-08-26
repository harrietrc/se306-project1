#include "Visitor.h"
#include "std_msgs/String.h"

/**
*	@brief Class for the Doctor nodes.
*/
class Doctor : public Visitor
{
	protected:
		bool doHeal();
		bool doHospitalise();
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		
};
