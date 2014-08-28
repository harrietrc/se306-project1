#include "Visitor.h"

/**
*	@brief Class for Nurse nodes.
*/
class Nurse2 : public Visitor {

	private:
		bool readyToHospitalise = false;
	
	protected:
		void doHospitalise();
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		Nurse2() {
			originName = "Nurse2Origin";
		}

};