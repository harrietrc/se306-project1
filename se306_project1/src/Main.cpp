#include "AgentFactory.h"

#include "ros/ros.h"

int main(int argc, char** argv)
//int main()
{
    AgentFactory af1;
    af1.createMockAgent();
    return 0;
}
