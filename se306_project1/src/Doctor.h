#include "Visitor.h"
#include "std_msgs/String.h"

/**
*	@brief Class for the Doctor nodes.
*/
class Doctor : public Visitor
{
	private:
		bool healing = false;
		bool readyToHeal = false;
		bool hospitalise = false;
		bool readyToHospitalise = false;
		
	protected:
		bool doHeal(std::string residentCheckpoint);
		bool doHospitalise(std::string residentCheckpoint);
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		
};