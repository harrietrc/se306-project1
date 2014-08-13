#include "Agent.h"

class Resident : public Agent
{
	protected:
		int health;
		int boredom;
		int hunger;
		
		//goal pose and orientation
		double goal_x;
		double goal_y;
		double goal_angle;

		bool running;
		bool isSet;

		//current pose and orientation of the robot
		double cur_angle;

		int cc; //current_checkpoint = 0;

		bool is_called; 

		std::pair<double,bool> goal_pair;
		std::pair<double, double> ret;	

		int checkpoints[5][2] = {  
		{30, 25}, 
		{30, 42}, 
		{12, 42},
		{30, 42},
		{30, 25}  
		};

	public:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
		int run(int argc, char **argv);
		void doctor_callback(se306_project1::DoctorMsg msg);
		std::pair <double,bool> calc_goal(double goal_x, double goal_y, double cur_angle, double px, double py);
		std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
		void randomCheckpointCallback(const ros::TimerEvent&);

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