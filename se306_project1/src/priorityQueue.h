/*
 * priorityQueue.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mustafa
 */

#include <string>

#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_


enum residentStates {hunger,healthLow,bored,emergency,tired,caregiver,friends,medication,idle};
/*std::string stateToString(residentStates currentState) {
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
		case (hunger):
				return "hunger";
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
}
*/
class priorityQueue {
public:
	priorityQueue();
	virtual ~priorityQueue();
	void addToPQ(residentStates currentState);
	residentStates popFromPQ();
	residentStates checkCurrentState();
private:
	bool isStateInPQ(residentStates currentState);
	void removeState(residentStates unwantedState);

};

#endif /* PRIORITYQUEUE_H_ */
