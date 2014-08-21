/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
* 	@todo Shift all common functionality (such as navigation) and properties to this class from subclasses.
*	@extends Agent
*/
class Agent
{
	protected:
		//velocity of the robot
		double linear_x = 0; /*!< Linear velocity of the robot */
		double angular_z = 0; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::pair<double, double> currentCheckpoint = std::make_pair(30,25);

		//shortestPath
		std::vector<std::pair<double,double> > shortestPath;
		int shortestPathIndex = 1;
		bool isFacingCorrectly = false;

		//moving status of the robot
		bool isMoving = true;

		int robot_id; /*!< Robot's ID */
};
