#include "Agent.h"

/**
*	@brief Causes the visitor to visit the resident, as based on a timer.
*	@param The timer that calls this callback automatically generates a TimerEvent.
*	@param startTimme The hour to start the periodic visits
*	@param endTime The hour to end the periodic visits.
*	@remarks If start time and end time are unnecessary, they can be removed
*/
void Visitor::doTimedVisit(const ros::TimerEvent&, int startTime, int endTime) {

}

/**
*	@brief Visit the resident.
*	May be simple - e.g. toggling visitor visibility, or moving to the door.
*/
void Visitor::visitResident() {

}

/**
*	@brief Initiates conversation with the resident.
*	@returns true if the behaviour completes successfully.
*/
bool Visitor::doConverse() {
	return true;
}

