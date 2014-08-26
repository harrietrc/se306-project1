#include <stdlib.h>
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"
#include <vector>

class CheckPointGraph {

	public:
		void makeGraph();
		void checkpointMap();
		void getShortestPath(std::string startName, std::string endName);
		
};