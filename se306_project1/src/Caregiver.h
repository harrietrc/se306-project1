#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver : public Visitor {

	protected:
		void delegate(se306_project1::ResidentMsg msg);
		bool doMoralSupport();
		bool doShowerSupport();
		bool doExerciseSupport();
		bool doEatSupport();

	public:
		int run(int argc, char *argv[]);
};
