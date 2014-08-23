/**
*	@brief Superclass for all 'agents' - i.e. Assistants, Visitors, and the Resident.
* 	@todo Shift all common functionality (such as navigation) and properties to this class from subclasses.
*	@extends Agent
*/
class Agent
{
	public:
		Agent(){
			linear_x = 0;
			angular_z = 0;
			px = 0;
			py = 0;
			currentAngle = 0;
			currentCheckpoint = std::make_pair(30,25);
			shortestPath.push_back(currentCheckpoint);
			isMoving = true;
			isFacingCorrectly = false;
			shortestPathIndex = 0;
			checkpointAngle = 0;
			isClockwise = true;
			robot_id = 0;

		}
	protected:
		//velocity of the robot
		double linear_x; /*!< Linear velocity of the robot */
		double angular_z; /*!< Angular velocity of the robot */
	
		//pose of the robot
		double px; /*!< x position of the robot */
		double py; /*!< y position of the robot */
		double currentAngle; /*!< angle of the robot*/

		//current checkpoint
		std::pair<double, double> currentCheckpoint;

		//shortestPath
		std::vector<std::pair<double,double> > shortestPath;
		int shortestPathIndex;
		bool isFacingCorrectly;

		//moving status of the robot
		bool isMoving;

		int robot_id; /*!< Robot's ID */

		double checkpointAngle;
		bool isClockwise;
};
