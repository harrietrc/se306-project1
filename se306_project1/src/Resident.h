#include "Agent.h"

class Resident : public Agent
{
	protected:
		int health;
		int boredom;
		int hunger;

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);

		// Return type of robot
		// MIGHT HAVE TO RETURN A STRING BECAUSE ROS DOESN'T SUPPORT ENUM IN MESSAGES
		//Type get_Type()
	
		// Get id of robot
		//int get_id(){
	
		// Wakes up
		//void wake_up()
	
		// Eat
		//void eat()
	
		// Takes medicine
		//void take_medicine()
	
		// Accepts entertainment
		//void accept_entertainment()
		
};
