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

class priorityQueue {
public:
	priorityQueue();
	virtual ~priorityQueue();
	void addToPQ(residentStates currentState);
	residentStates popFromPQ();
	std::string checkCurrentState();

private:
	bool isStateInPQ(residentStates currentState);
	void removeState(residentStates unwantedState);
	std::string stateConvertString(residentStates currentState);

};

#endif /* PRIORITYQUEUE_H_ */
