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
	std::string type = "R0";
	return processManager.nodeProcess(type);
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

