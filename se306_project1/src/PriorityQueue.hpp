/*
* Everything defined and implemented in header file for now
* because when i move implementation into c++, ros freaks out.
* - mustafa
*/


#include <iostream>
#include <string.h>
#include <queue>
#include <iomanip>

using namespace std;


// all the statuses the resident implements
enum STATUS {SILL,ILL,HUNGRY,TIRED,BORED,HEALTHCARE,IDLE};

// this struct will be stored in the PQ
struct state { 
	int priority;
	STATUS status;
};


class CompareStates { 
	public:
		bool operator()(state& s1, state& s2) // returns true if s1 priority is higher than s2 priority, used in the priorityQueue template for c++
		{
			if (s1.priority > s2.priority) {
				return true;
			}
			return false;

		}
};

priority_queue<state,vector<state>,CompareStates> myQueue;


/*
* PriorityQueue implemented as a singleton, below is 
* a code snippet of how to add to the Queue
* #include "PriorityQueue.hpp"
* ....
* ....
* PriorityQueue *s1 = PriorityQueue::getInstance();
* s1->add(HUNGRY);
* s1->checkIfIn(SILL)
* s1->pop()
* ....
*/
class PriorityQueue {
public:
	static PriorityQueue * getInstance(){
		static PriorityQueue * instance; // making sure only one instance is ever alive
		return instance;
	};

	// returns the highest Priority in queue
	STATUS checkHighest() {
		if (!myQueue.empty()) {
			return myQueue.top().status;
		} else {
			return IDLE;
		}
	}

	// removes top
	STATUS remove() {
		if (!myQueue.empty()) {
			state s1 = myQueue.top();
			myQueue.pop();
			return s1.status;
		} else {
			return IDLE;
		}	
	}

	// checks if a status is already inside the queue
	// it works by deconstructing the queue and rebuilding
	// this is because priority_queue doesn't allow for iteration
	// and it doesn't have any methods to check if something is inside
	bool checkIfIn(STATUS status) {
		priority_queue<state,vector<state>,CompareStates> TempQueue; 
		state s1;
		bool inside = false;
		while(!myQueue.empty()) {
			s1 = myQueue.top();
			if (s1.status == status) {
				inside = true;
			}
			TempQueue.push(s1);
			myQueue.pop();
		}
		
		while(!TempQueue.empty()) {
			s1 = TempQueue.top();
			myQueue.push(s1);
			TempQueue.pop();
		}
		
		return inside;
	}

	// adding a status into the queue
	void add(STATUS status) {
		
		if (!checkIfIn(status)) { // to make sure a status isn't repeated
			state s1;
			int priority = 0; 
			switch (status)
			{
				case SILL:
					priority = 1;
					break;
				case ILL:
					priority = 2;
					break;
				case HUNGRY:
					priority = 4;
					break;
				case TIRED:
					priority = 5;
					break;
				case BORED:
					priority = 6;
					break;
				case HEALTHCARE:
					priority = 3;
					break;
				case IDLE:
					break;
			}
			if (priority > 0) { // just incase idle was selected
				s1.priority = priority;
				s1.status = status;
				myQueue.push(s1);
			}
		}
	};
	void printAll() { // a testing method
		state s1;
		myQueue.empty();
		while( !myQueue.empty()) {
			s1 = myQueue.top();
			myQueue.pop();
			ROS_INFO("%d",s1.priority);
			switch (s1.status)
			{
				case SILL:
					ROS_INFO("SILL");
					break;
				case ILL:
					ROS_INFO("ILL");
				case HUNGRY:
					ROS_INFO("HUNGRY");
					break;
				case TIRED:
					ROS_INFO("TIRED");
					break;
				case BORED:
					ROS_INFO("BORED");
					break;
				case HEALTHCARE:
					ROS_INFO("HEALTHCARE");
					break;
				case IDLE:
					break;
			}
		}
		
	};



private:
	PriorityQueue();
	~PriorityQueue();	
};

