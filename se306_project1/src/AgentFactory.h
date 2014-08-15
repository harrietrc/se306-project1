/*
 * AgentFactory.h
 *
 *  Created on: 13/08/2014
 *      Author: john
 */

#include "ros/ros.h"

namespace AgentConst {
	enum AgentType { RESIDENT, ASSISTANT, DOCTOR };
}

class AgentFactory {
public:
//	AgentFactory();
//	virtual ~AgentFactory();
	void createMockAgent();
	int createAgent(AgentConst::AgentType agentType, int nodeNumber);
};
