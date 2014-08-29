#include "Visitor.h"

/**
*	@brief Superclass for visitors - i.e. Caregivers, Nurses, Doctors, Friends, Relatives
*/
class Caregiver : public Visitor {

	private:
		// bool variables
		bool hasShowered;
		bool hasExercised;
		bool atResident;
		bool exercise(se306_project1::ResidentMsg msg);
		bool shower(se306_project1::ResidentMsg msg);

		// methods
		void spin();
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		Caregiver() : Visitor(){
			originName = "Caregiver2Origin";
			currentCheckpoint = std::make_pair(-6, -42); // Could just use g.getCoords(originName)
			px = -6;
			py= -42;
		}
};
