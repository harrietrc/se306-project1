/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
* 	@todo Shift all common functionality (such as navigation) and properties to this class from subclasses.
*	@extends Agent
*/
class Agent
{
	protected:
		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */

		// Enumeration of type of robot
		enum Type{FRIEND, RELATIVE, DOCTOR, NURSE, CAREGIVER, ASSISTANT, RESIDENT}; /*!< Enumeration of the types of robots */
	
		int robot_id; /*!< Robot's ID */

	public:
		
		/**
		*	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
		*	@param msg Odometry message from odom topic
		*/
		virtual void StageOdom_callback(nav_msgs::Odometry msg) = 0;

		/**
		*	@brief Virtual callback function to process laser scan messsages.
		*	@param msg Single scan from a planar laser range finder
		*/
		virtual void StageLaser_callback(sensor_msgs::LaserScan msg) = 0;

		/**
		*	@brief Main function that controls agent events.
		*/
		virtual int run(int argc, char *argv[]) = 0;
};
