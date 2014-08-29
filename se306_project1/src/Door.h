/**
*	@brief Class for the Door nodes.
*/
class Door
{
	private:
		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		
		bool opened = false;
        int waitTime = 0;
		
		std::string resStateWhenOpened = "";
		
	protected:
		void StageOdom_callback(nav_msgs::Odometry msg);
		void delegate(se306_project1::ResidentMsg msg);
		void open();
		void close(int waitTime);

	public:
		int run(int argc, char *argv[]);
		
};
