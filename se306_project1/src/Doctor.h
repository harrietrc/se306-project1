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
		bool hospitalised = false;
		bool readyToHospitalise = false;
		bool isHealed = false;

		ros::Publisher Doctor_state_pub;

	protected:
		void doHeal(se306_project1::ResidentMsg msg);
		void hospitalise(se306_project1::ResidentMsg msg);
		void delegate(se306_project1::ResidentMsg msg);

	public:
		int run(int argc, char *argv[]);
		Doctor() {
			originName = "DoctorOrigin";
			currentCheckpoint.first = -33;
			currentCheckpoint.second = -42;
		}
		
};
