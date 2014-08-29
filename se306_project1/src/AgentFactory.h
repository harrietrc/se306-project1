/*
 * AgentFactory.h
 *
 *  Created on: 13/08/2014
 *      Author: john
 */

#include "ros/ros.h"

namespace AgentConst {
	enum AgentType { RESIDENT, ASSISTANT, CAREGIVER, DOCTOR, NURSE, FRIENDS, DOOR };
}

/**
*	@brief Class for instantiating agents dynamically.
*/
class AgentFactory {
public:

	void createMockAgent();
	int createAgent(AgentConst::AgentType agentType, int nodeNumber);
	
};
