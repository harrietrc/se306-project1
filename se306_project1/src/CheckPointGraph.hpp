#include <stdlib.h>
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"
#include <vector>

class CheckPointGraph {

	public:
		CheckPointGraph() {
			checkpointMap(); // Apparently calling functions in the constructor is less bad in C++ than in Java...
			makeGraph(); // ditto, but should consider moving. These are here for convenience.
		}
		void makeGraph();
		void checkpointMap();
		std::vector<std::pair<double, double> > shortestPath(std::string startName, std::string endName);
		std::vector<std::pair<double, double> > shortestPathAsDoubles(std::string start, std::string end);
};