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

	public:
		int run(int argc, char *argv[]);
		
};
