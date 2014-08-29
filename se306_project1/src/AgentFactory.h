/*
 * AgentFactory.h
 *
 *  Created on: 13/08/2014
 *      Author: john
 */

#include "ros/ros.h"

/**
*	@brief Enum representing the different types of agent.
*/
namespace AgentConst {
	enum AgentType { RESIDENT, ASSISTANT, ASSISTANT1, CAREGIVER, DOCTOR, NURSE, NURSE1, FRIEND, FRIEND1, FRIEND2,  DOOR };
}

/**
*	@brief Class for instantiating agents dynamically.
*/
class AgentFactory {
public:

	void createMockAgent();
	int createAgent(AgentConst::AgentType agentType, int nodeNumber);
	
};
