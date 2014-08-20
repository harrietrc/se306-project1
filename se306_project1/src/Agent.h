/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
*	Contains common navigation functionality and properties.
*/
class Agent
{
	protected:
		/* -- Positional attributes -- */

		std::pair<double, double> origin; /*!< The initial position of the agent.  @note Replaces checkpoint[0] */
		double goal_x; /*!< The x position of the robot's goal @note Left in to better accommodate for interruption */
		double goal_y; /*!< The y position of the robot's goal @note Left in to better accommodate for interruption */
		double goal_angle; /*!< The angle that the robot must face to approach the goal, defined in reference to the co-ordinate system */
		double px; /*!< The agent's current x position */
		double py; /*!< The agent's current y position */
		double cur_angle; /*!< The robot's current facing in reference to the co-ordinate system */

		/* -- Navigational/movement attributes -- */

		double linear_x; /*!< Agent's linear velocity. Published to stage. */
		double angular_z; /*!< Agent's rotational velocity around the z axis. Published to stage. */
		bool isSet = false; /*!< True if agent is facing in the right direction (?) */

		/* -- Checkpoints and graph -- */

		/**
		*	@brief Set of checkpoints that the nodes can move to in the map.
		* 	Gives x position and y position for each co-ordinate
		*/
		int checkpoints[11][2] = {  
		{30, 25},
		{30, 7}, 
		{40, 7},
		{40, 8},
		{38,8},
		{38,7},
		{40,7},
		{30,7},
		{30, 25},
		{34,20},
		{30, 25}
		};

		/* -- Navigation and movement functions -- */

		std::pair<double, double> movePath(); /*!< @note No parameters as long as the agent's path is a field. */
		std::pair<double, double> move(); /*!< @note Same return type as movePath() because move() generates the return value for movePath(). */
		double calc_goal_angle();
		std::vector<std::pair<double, double> > shortestPath(std::string startName, std::string endName);

		void makeGraph();
		void checkpointMap();

		/* -- Communication and co-ordination -- */

		virtual void delegate(se306_project1::ResidentMsg msg); /*!< Callback that calls callbacks */


		virtual int run(int argc, char *argv[]) = 0; /*!< Pure virtual function, made accessible through the non-member main() function */

	public:
		
		void StageOdom_callback(nav_msgs::Odometry msg);
		void StageLaser_callback(sensor_msgs::LaserScan msg);
};
