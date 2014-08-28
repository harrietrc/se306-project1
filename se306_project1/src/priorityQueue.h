/*
 * priorityQueue.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mustafa
 */

#include <string>

#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_



enum residentStates {hungry,healthLow,bored,emergency,tired,caregiver,friends,medication,idle};

class priorityQueue {
public:
	priorityQueue();
	virtual ~priorityQueue();
	void addToPQ(residentStates currentState);
	residentStates popFromPQ();
	std::string checkCurrentState();
	void removeState(residentStates unwantedState);

private:
	bool isStateInPQ(residentStates currentState);
	std::string stateConvertString(residentStates currentState);

};

#endif /* PRIORITYQUEUE_H_ */
