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
		std::pair<double, double> currentCheckpoint;

		//shortestPath
		std::vector<std::pair<double,double> > shortestPath;
		int shortestPathIndex;
		bool isFacingCorrectly;

		//moving status of the robot
		bool isMoving;

		int robot_id; /*!< Robot's ID */

		double checkpointAngle;
		double angleDiffCheckpointCurrent;
		bool isClockwise;
};
