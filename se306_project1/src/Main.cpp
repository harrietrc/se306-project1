#include "AgentFactory.h"

#include "ros/ros.h"

std::map<AgentConst::AgentType, int> nodeList;

void initializeNodeList();

/**
*	@brief Responsible for ensuring that nodes are initialised
*	Communicates with AgentFactory
*/
int main(int argc, char** argv) {
	initializeNodeList();

	AgentConst::AgentType agentType = AgentConst::RESIDENT;
	AgentFactory agentFactory;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType);
	std::cout << "now we have " << nodeList[agentType] << " resident(s)\n";

	agentType = AgentConst::ASSISTANT;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType);
	std::cout << "now we have " << nodeList[agentType] << " assistant(s)\n";

	agentType = AgentConst::DOCTOR;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType);

	std::cout << "now we have " << nodeList[agentType] << "doctor(s)\n";	
	
	return 0;
}

/**
*	@brief Initialises the list of nodes.
*/
void initializeNodeList() {
	nodeList[AgentConst::RESIDENT] = 0;
	nodeList[AgentConst::ASSISTANT] = 0;
	nodeList[AgentConst::DOCTOR] = 0;
}
