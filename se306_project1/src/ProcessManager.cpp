/*
 * ProcessManager.cpp
 *
 *  Created on: 13/08/2014
 *      Author: john
 */
#include <stdlib.h>
#include "ProcessManager.h"
#include <iostream>

#include "ros/ros.h"
using namespace std;

int ProcessManager::nodeProcess(std::string executableName, int nodeNumber) {
	pid_t child = 0;
	child = fork();
	if (child < 0) {
		fprintf(stderr, "process failed to fork\n");
		return 0;
	}
	if (child == 0) {
		std::stringstream val;
		val << nodeNumber;
		const char* argNumConst = val.str().c_str();

		char* argNum = const_cast<char*>(argNumConst);

		char* execName = const_cast<char*>(executableName.c_str());

		std::stringstream pathm9;
		pathm9 << "./bin/" << executableName;

		char* commands[] = { execName, (char *)0 };

		execvp(pathm9.str().c_str(), commands);

		return 1;
	} else {
		return 1;
	}
}
