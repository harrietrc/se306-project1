/*
 * AgentFactory.cpp
 *
 *  Created on: 13/08/2014
 *      Author: john
 */

#include "AgentFactory.h"
#include "ProcessManager.h"
#include <iostream>

using namespace std;

/**	
*	@brief Creates a mock agent - for testing
*/
void AgentFactory::createMockAgent() {
	cout << "I will create a mock agent";
}

int AgentFactory::createAgent(AgentConst::AgentType agentType, int nodeNumber) {
	cout << "creating agent using process manager, agent type: " << agentType << endl;
	ProcessManager processManager;
	std::string stringType = "";

	std::string agentTypes[] = { "Resident", "Assistant", "Doctor", "Caregiver" };

	stringType = agentTypes[(int)agentType];

	return processManager.nodeProcess(stringType, nodeNumber);
}

