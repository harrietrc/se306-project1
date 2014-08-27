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
	// TODO Auto-generated constructor stub

}

priorityQueue::~priorityQueue() {
	// TODO Auto-generated destructor stub
}

bool priorityQueue::isStateInPQ(residentStates currentState) {


	for (unsigned i=0; i<PQ.size();i++) {
		if (PQ.at(i).state == currentState) {
			return true;
		}
		if (i > 7 ) {
			break;
		}
	}
	return false;

}


residentStates priorityQueue::checkCurrentState() {
	return PQ.back().state;
}

void priorityQueue::addToPQ(residentStates currentState) {
	if (isStateInPQ(currentState)) { // if state is in PQ, don't add again
		return;
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
	case (friends):
			currResidentStatus.priority = 4;
			break;
	case (hunger):
			currResidentStatus.priority = 5;
			break;
	case (tired):
			currResidentStatus.priority = 6;
			break;
	case (bored):
			currResidentStatus.priority = 7;
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

				if (i > 7 ) {
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

