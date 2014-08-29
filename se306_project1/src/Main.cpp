#include "AgentFactory.h"

#include "ros/ros.h"

std::map<AgentConst::AgentType, int> nodeList;

void initializeNodeList();
int getNodeNumber();

/**
*	@brief Responsible for ensuring that nodes are initialised
*	Communicates with AgentFactory
*/
int main(int argc, char** argv) {
	initializeNodeList();

	AgentFactory agentFactory;

	AgentConst::AgentType agentType = AgentConst::RESIDENT;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	//std::cout << "now we have " << nodeList[agentType] << " resident(s)\n";

	agentType = AgentConst::ASSISTANT;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	//std::cout << "now we have " << nodeList[agentType] << " assistant(s)\n";

	agentType = AgentConst::CAREGIVER;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());

	agentType = AgentConst::DOCTOR;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());

	agentType = AgentConst::NURSE;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());

	agentType = AgentConst::FRIENDS;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());

	agentType = AgentConst::DOOR;
	nodeList[agentType] = nodeList[agentType] + agentFactory.createAgent(agentType, getNodeNumber());

	//std::cout << "now we have " << nodeList[agentType] << "doctor(s)\n";
	
	return 0;
}

/**
*	@brief Initialises the list of nodes.
*/
void initializeNodeList() {
	nodeList[AgentConst::RESIDENT] = 0;
	nodeList[AgentConst::ASSISTANT] = 0;
	nodeList[AgentConst::DOCTOR] = 0;
	nodeList[AgentConst::CAREGIVER] = 0;
	nodeList[AgentConst::DOOR] = 0;
	nodeList[AgentConst::NURSE] = 0;
	nodeList[AgentConst::FRIENDS] = 0;
}

int getNodeNumber() {
	return nodeList[AgentConst::RESIDENT] + nodeList[AgentConst::ASSISTANT] + nodeList[AgentConst::DOCTOR];

}
