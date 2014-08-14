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
		const char* argNum = val.str().c_str();

		std::cout<< "starting process with name " << executableName << " and nodeNumber " << argNum << "\n";
		std::string path = "/home/john/se306-project1/se306_project1/bin/" + executableName;

		execl(path.c_str(), executableName.c_str(), argNum, (char*)0);
		//wait(NULL);
		return 1;
	} else {
		return 1;
	}
}
