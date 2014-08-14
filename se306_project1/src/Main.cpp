#include "AgentFactory.h"

#include "ros/ros.h"

std::map<AgentConst::AgentType, int> nodeList;

void initializeNodeList();

int main(int argc, char** argv) {
	initializeNodeList();

	AgentConst::AgentType agentType = AgentConst::RESIDENT;
	AgentFactory agentFactory;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType);

	std::cout << "now we have " << nodeList[agentType] << "resident(s)\n";

	agentType = AgentConst::ASSISTANT;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType);

	std::cout << "now we have " << nodeList[agentType] << "assistant(s)\n";

	return 0;
}

void initializeNodeList() {
	nodeList[AgentConst::RESIDENT] = 0;
	nodeList[AgentConst::DOCTOR] = 0;
	nodeList[AgentConst::RESIDENT] = 0;
}
