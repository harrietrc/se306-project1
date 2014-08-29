#include "Visitor.h"

/**
*	@brief Class for Nurse nodes.
*/
class Nurse1 : public Visitor {

	private:
		bool readyToHospitalise = false;
	
	protected:
		void doHospitalise(se306_project1::ResidentMsg msg);
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		Nurse1() {
			originName = "Nurse1Origin";
		}

};
