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


int ProcessManager::nodeProcess(std::string executableName) {
	pid_t child = 0;
	child = fork();
	if (child < 0) {
		fprintf(stderr, "process failed to fork\n");
		return 0;
	}
	if (child == 0) {
		char* args[1];
		args[0] = "1";
		std::cout<< "starting process with name " << executableName << "\n";
		std::string path = "/afs/ec.auckland.ac.nz/users/g/s/gsam265/unixhome/se306-project1/se306_project1/bin/" + executableName;
		execl(path.c_str(), executableName.c_str(), "69", (char*)0);
		//wait(NULL);
		return 1;
	} else {
		return 1;
	}
}

