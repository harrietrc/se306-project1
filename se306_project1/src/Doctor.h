#include "Visitor.h"
#include "std_msgs/String.h"

/**
*	@brief Class for the Doctor nodes.
*/
class Doctor : public Visitor
{
	private:
		bool healing = false; /*!< True if the doctor is currently healing the resident */
		bool readyToHeal = false; /*!< True if the doctor is ready to heal th resident - i.e. they are adjacent to them */
		bool hospitalised = false; /*!< True if the resident is in hospital */
		bool readyToHospitalise = false; /*!< True if the doctor is ready to take the resident to hospital (must be adjacent to them) */
		bool isHealed = false; /*!< True if the resident has been healed */

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
