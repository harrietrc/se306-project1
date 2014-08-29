#include <stdlib.h>
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"
#include <vector>

/**
*	@brief Class representing the checkpoints and the paths between them.
*	Also used in the calculation of the shortest path between checkpoints.
*/
class CheckPointGraph {

	public:
		CheckPointGraph() {
			checkpointMap(); // Apparently calling functions in the constructor is less bad in C++ than in Java...
			makeGraph(); // ditto, but should consider moving. These are here for convenience.
		}
		void makeGraph();
		void checkpointMap();
		std::vector<std::pair<double, double> > shortestPath(std::string startName, std::string endName);
		std::string getCheckpointName(std::pair<double, double> coords);
		std::pair<double,double> getCoords(std::string name);

};
