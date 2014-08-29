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

/**
*	@brief Creates an agent using the process manager and the provided type and number.
*	@param agentType The type of node - e.g. Doctor, Resident, etc.
*	@param nodeNumber A number that uniquely identifies the node.
*/
int AgentFactory::createAgent(AgentConst::AgentType agentType, int nodeNumber) {
	cout << "creating agent using process manager, agent type: " << agentType << endl;
	ProcessManager processManager;
	std::string stringType = "";

	std::string agentTypes[] = { "Resident", "Assistant", "Assistant1", "Caregiver", "Doctor", "Nurse", "Nurse1", "Friend", "Friend1", "Friend2" };

	stringType = agentTypes[(int)agentType];

	return processManager.nodeProcess(stringType, nodeNumber);
}

