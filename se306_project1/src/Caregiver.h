#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver : public Visitor {

	private:
		// bool variables
		bool hasShowered;
		bool hasExericesed;
		bool atResident;
		bool exercise();
		bool shower();

		// methods
		void spin();
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
};
