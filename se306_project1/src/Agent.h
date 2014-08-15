/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
* 	@todo Shift all common functionality (such as navigation) and properties to this class from subclasses.
*	@extends Agent
*/
class Agent
{
	protected:
		//velocity of the robot
		double linear_x;
		double angular_z;
	
		//pose of the robot
		double px;
		double py;

		// Enumeration of type of robot
		enum Type{FRIEND, RELATIVE, DOCTOR, NURSE, CAREGIVER, ASSISTANT, RESIDENT};
	
		int robot_id;

	public:
		virtual void StageOdom_callback(nav_msgs::Odometry msg) = 0;
		virtual void StageLaser_callback(sensor_msgs::LaserScan msg) = 0;
		virtual int run(int argc, char *argv[]) = 0;
};
