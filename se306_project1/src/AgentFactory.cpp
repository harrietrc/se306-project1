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


void AgentFactory::createMockAgent() {
	cout << "I will create a mock agent";
}

int AgentFactory::createAgent(AgentConst::AgentType agentType) {
	cout << "creating agent using process manager, agent type: " << agentType << endl;
	ProcessManager processManager;
	std::string stringType = "R0";

	std::string agentTypes[] = { "RESIDENT", "ASSISTANT", "DOCTOR" };

	stringType = agentTypes[(int)agentType];

	return processManager.nodeProcess(stringType);
}

//int main() {return 0;}
//AgentFactory::AgentFactory() {
//	// TODO Auto-generated constructor stub
//
//}
//
//AgentFactory::~AgentFactory() {
//	// TODO Auto-generated destructor stub
//}

