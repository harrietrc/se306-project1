#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver2 : public Visitor {

	private:
		// bool variables
		bool hasShowered; /*!< true if the resident has showered */
		bool hasExercised; /*!< true if the resident has exercised */
		bool atResident; /*!< True if the assistant is at the resident's position */
		bool exercise(se306_project1::ResidentMsg msg);
		bool shower(se306_project1::ResidentMsg msg);

		// methods
		void spin();
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		Caregiver2() {
			originName = "Caregiver2Origin";
		}
};
