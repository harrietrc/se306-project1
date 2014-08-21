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
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::pair<int, int> currentCheckpoint = std::make_pair(30,25);

		//shortestPath
		std::vector<std::pair<double,double>> shortestPath = {{30,25},{35,30},{35,25}};
		int shortestPathIndex = 1;

		//moving status of the robot
		bool isMoving = false;

		int robot_id; /*!< Robot's ID */
};
