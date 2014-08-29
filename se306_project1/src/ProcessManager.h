/*
 * ProcessManager.h
 *
 *  Created on: 13/08/2014
 *      Author: john
 */

#include <string>

/**
*	@brief Class for managing node processes. 
*	Should keep track of nodes and be able to instantiate them.
*/
class ProcessManager {
public:
	int nodeProcess(std::string executableName, int nodeNumber);
};
