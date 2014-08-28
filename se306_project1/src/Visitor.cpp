#include "Visitor.h"
#include "ros/ros.h"
#include "math.h"

/**
*	@brief Visit the resident.
*	May be simple - e.g. toggling visitor visibility, or moving to the door.
*	@returns true if this has moved next to the Resident
*/
bool Visitor::visitResident(std::string residentCheckpoint) {
	double lastCheckpointX = shortestPath.at(shortestPath.size()-1).first;
	double lastCheckpointY = shortestPath.at(shortestPath.size()-1).second;

	double distanceFromCheckpoint = sqrt(pow((lastCheckpointX - px),2) + pow((lastCheckpointY - py),2));
	
	move(residentCheckpoint);
	if (distanceFromCheckpoint < 0.5) { // next to resident
		return true;
	}
	
	return false;
}

/**
*	@brief Initiates conversation with the resident.
*	@returns true if the behaviour completes successfully.
*/
bool Visitor::doConverse() {
	// converse for an hour?
	int tnow = ros::Time::now().toSec(); // The simulation time now
	int finishConvo = tnow + 1; // one hour later
	
	ROS_INFO("Starting conversation...");
	while (tnow < finishConvo) {
		tnow = ros::Time::now().toSec(); // The simulation time now
	}

	// convo should be done and visitor should leave
	finishedConvo = true;
	
	return finishedConvo;
}