/*
 * This class implements a reverse order PQ so its 7,6,5 not 5,6,7
 * This is done to make use of the 0(1) methods that c++ vector provide
 */

#include "priorityQueue.h"
#include <vector>
#include <iostream>
#include "ros/ros.h"

struct statusObj { //
int priority;
residentStates state;
};

std::vector<statusObj> PQ;

priorityQueue::priorityQueue() {
	addToPQ(tired);

}

priorityQueue::~priorityQueue() {
	// TODO Auto-generated destructor stub
}

bool priorityQueue::isStateInPQ(residentStates currentState) {


	for (unsigned i=0; i<PQ.size();i++) {
		if (PQ.at(i).state == currentState) {
			return true;
		}
		if (i > 8 ) {
			break;
		}
	}
	return false;

}


std::string priorityQueue::checkCurrentState() {
	if (PQ.empty()) {
		return stateConvertString(idle);
	} else {
		return stateConvertString(PQ.back().state);
	}
}

void priorityQueue::removeState(residentStates unwantedState) {

	if (!isStateInPQ(unwantedState)) {
		return;
	}
	for (unsigned i=0; i<PQ.size();i++) {
		if (PQ.at(i).state == unwantedState) {
			PQ.erase(PQ.begin() + i);
			return;
		}
	}

}

void priorityQueue::addToPQ(residentStates currentState) {
	if (isStateInPQ(currentState)) { // if state is in PQ, don't add again
		return;
	}

	if (currentState == emergency) {
		removeState(healthLow);
	}
	if (currentState == healthLow) {
		if(isStateInPQ(emergency)) {
			return;
		}
	}



	statusObj currResidentStatus;
	currResidentStatus.state = currentState;

	switch(currentState) // assigning the priority
	{
	case (emergency):
			currResidentStatus.priority = 1;
			break;
	case (healthLow):
			currResidentStatus.priority = 2;
			break;
	case (caregiver):
			currResidentStatus.priority = 3;
			break;
	case(medication):
			currResidentStatus.priority = 4;
			break;
	case (friends):
			currResidentStatus.priority = 5;
			break;
	case (hungry):
			currResidentStatus.priority = 6;
			break;
	case (tired):
			currResidentStatus.priority = 7;
			break;
	case (bored):
			currResidentStatus.priority = 8;
			break;
	case (idle):
			break;
	}

	if (currentState != idle) {

		if (PQ.empty()) { // if PQ is empty, add to the back.

			PQ.push_back(currResidentStatus);

		} else {

			bool statusInserted = false;

			for (unsigned i=0; i<PQ.size();i++) {

				if (currResidentStatus.priority > PQ.at(i).priority) {

					PQ.insert(PQ.begin() + i, currResidentStatus);
					statusInserted = true;
					break;

				}

				if (i > 8 ) {
					break;
				}

			}
			if (statusInserted == false) {
				PQ.push_back(currResidentStatus);
			}
		}

	}
}

residentStates priorityQueue::popFromPQ() {

	if (PQ.empty()) {
		return idle;

	} else {
		residentStates state = PQ.back().state;
		PQ.pop_back();
		return state;
	}
}

std::string priorityQueue::stateConvertString(residentStates currentState){
	switch(currentState) // assigning the priority
		{
		case (emergency):
				return "emergency";
				break;
		case (healthLow):
				return "healthLow";
				break;
		case (caregiver):
				return "caregiver";
				break;
		case(medication):
				return "medication";
				break;
		case (friends):
				return "friends";
				break;
		case (hungry):
				return "hungry";
				break;
		case (tired):
				return "tired";
				break;
		case (bored):
				return "bored";
				break;
		case (idle):
				return "idle";
				break;
		}
	return "Can't convert!!"; //
}

