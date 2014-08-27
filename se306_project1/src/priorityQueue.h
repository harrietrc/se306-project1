/*
 * priorityQueue.h
 *
 *  Created on: Aug 27, 2014
 *      Author: mustafa
 */

#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_


enum residentStates {hunger,healthLow,bored,emergency,tired,caregiver,friends,idle};


class priorityQueue {
public:
	priorityQueue();
	virtual ~priorityQueue();
	void addToPQ(residentStates currentState);
	residentStates popFromPQ();
	residentStates checkCurrentState();
private:
	bool isStateInPQ(residentStates currentState);

};

#endif /* PRIORITYQUEUE_H_ */
