class Agent
{
	protected:
		//velocity of the robot
		double linear_x;
		double angular_z;
	
		//pose of the robot
		double px;
		double py;
		double theta;

		// Enumeration of type of robot
		enum Type{FRIEND, RELATIVE, DOCTOR, NURSE, CAREGIVER, ASSISTANT, RESIDENT};
	
		int robot_id;

	public:
		virtual void StageOdom_callback(nav_msgs::Odometry msg) = 0;
		virtual void StageLaser_callback(sensor_msgs::LaserScan msg) = 0;
		virtual int run(int argc, char **argv) = 0;
};
