#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"
#include "CheckPointGraph.hpp"

using namespace boost; // Useful for graphs

/** Array of the names of checkpoints. Necessary for the initialisation of the checkpoints vector. */
const char* nameArr[] = { 
		"cp0","cp1", "cp2"
};

/**
*	@brief Set of checkpoints that the nodes can move to in the map.
* 	Gives x position and y position for each co-ordinate
*/
int checkpoints[11][2] = {  
	{30, 25},
	{30, 7}, 
	{40, 7},
	{40, 8},
	{38,8},
	{38,7},
	{40,7},
	{30,7},
	{30, 25},
	{34,20},
	{30, 25}
};

std::vector<std::string> checkpointNames(begin(nameArr), end(nameArr)); /*!< Vector of checkpoint names. See nameArr[]. */

/* -- Graph -- */

typedef property<vertex_name_t, std::string> VertexProperty; /*!<  Will allow us to retrieve vertex names from vertex references */
typedef adjacency_list <vecS, vecS, undirectedS, VertexProperty> vector_graph_t; /*!< Graph of checkpoint names */
const int checkpointNum = checkpointNames.size(); /*!< Number of checkpoints. */
vector_graph_t g(checkpointNum); // Our graph
std::map<std::string, vector_graph_t::vertex_descriptor> indices; /*!< Map that corresponds checkpoint names to the vertices in the graph. */

/* -- Edges -- */

typedef std::pair <std::string, std::string> E;
E paths[] = { E ("cp0", "cp1"), E ("cp1", "cp2")}; /*!< Defines edges between checkpoints */

/* -- Map of names to co-ordinates -- */

typedef std::string CheckpointName; // Key
typedef std::pair<int, int> Checkpoint; // Value
typedef std::map<CheckpointName, Checkpoint> CheckpointMap; /*!< Map with checkpoint names as keys and checkpoint co-ordinates as values */
CheckpointMap c;

/* -- Pathing -- */

int cc; /*!< The index of the agent's current checkpoint in the path. */
std::vector<std::pair<double, double> > path; /*!< The agent's path to a specified goal. */

/**
 *	@brief Finds the shortest path between 2 checkpoints, and returns the path as co-ordinates.
 *	Uses breadth first search - Boost recommends this over Dijkstra's algorithm for graphs with uniformly weighted edges.
 * 	Sets path member variable.
 *	@param startName The name of the start checkpoint as a string (e.g. 'kitchen')
 *	@param endName The name of the goal checkpoint as a string (e.g. 'bathroom')
 */
std::vector<std::pair<double, double> > CheckPointGraph::shortestPath(std::string startName, std::string endName) {

	//Create vector to store the predecessors (can also make one to store distances)
  	std::vector<vector_graph_t::vertex_descriptor> p(boost::num_vertices(g));

  	// Get the descriptor for the source node
  	vector_graph_t::vertex_descriptor s = indices[startName]; // Shouldn't be hardcoded - pass start checkpoint

 	// Computes the shortest path 
 	breadth_first_search(g, s, visitor(make_bfs_visitor(record_predecessors(&p[0], on_tree_edge()))));

 	// Get the path back from the predecessor map
 	vector_graph_t::vertex_descriptor goal = indices[endName]; // As above, end checkpoint
 	std::vector<vector_graph_t::vertex_descriptor> path;
	vector_graph_t::vertex_descriptor current;

	current = goal;

	// This loop could be eliminated by pushing checkpoint names to a vector rather than vertex_descriptors. However,
	// I left it in because it seemed like good practice to create/retain the actual path of vertices, e.g. in case
	// there we should implement and want to access properties other than just vertex names.
	while(current!=s) {
   		path.push_back(current);
    	current = p[current]; // Predecessor of the current checkpoint in the path
	}

	// path.push_back(s); // BFS doesn't include the start node. 

	std::vector<std::pair<double, double> > a;

	for (int i=0; i<path.size(); i++) {
		// Get the vertex name from the graph's property map
		std::string cpn = boost::get(vertex_name_t(), g, path[i]); // adjacency_list vertex_descriptors are ints
		std::pair<double, double> coords = c[cpn]; // Get co-ordinates associated with checkpoint name
		a.push_back(coords);
	}
	
	std::reverse(a.begin(), a.end()); // as search starts from goal; we can access only predecessors, not successors

    return a;
}

/**
*	@brief Creates a graph of checkpoint names, provided the vector of names and the array of edges.
*	Uses boost's adjacency list.
*/
void CheckPointGraph::makeGraph() {

	// Fills the property 'vertex_name_t' of the vertices, allowing us to get the checkpoint name back when we have 
	// only a reference to the vertex (as will be the case when examining the shortest path). Also associates each
	// checkpoint name with a vertex descriptor.
	for(int i = 0; i < checkpointNum; i++)
	{
	  boost::put(vertex_name_t(), g, i, checkpointNames[i]); 
	  indices[checkpointNames[i]] = boost::vertex(i, g); 
	}

	// Add the edges. 
	for(int i = 0; i < sizeof(paths)/sizeof(paths[0]); i++)
	{
	  boost::add_edge(indices[paths[i].first], indices[paths[i].second], g);
	}

	// //Prints a pretty graph
	// std::ofstream ofs("test.dot");
    // write_graphviz(ofs, g); // dot -Tps test.dot -o outfile.ps	

}

/**
 *	@brief Associates checkpoint names with checkpoint co-ordinates
 *	To be used in conjunction with a graph of checkpoint names, representing paths between checkpoints. Could be replaced
 *	by adding the co-ordinates to bundled properties in the property map of the graph.
 */
void CheckPointGraph::checkpointMap() {

	// Convert array to pairs
	std::vector<std::pair<int,int> > vec;
	for (int i=0; i<checkpointNum; i++) {
		std::pair<int, int> p = std::make_pair(checkpoints[i][0],checkpoints[i][1]);
		vec.push_back(p);
	}

	// Add checkpoint name and checkpoint co-ordinates to the map
	for (int i=0; i<checkpointNum; i++) {
		c.insert(std::make_pair(CheckpointName(checkpointNames[i]), Checkpoint(vec[i])));
	}

}
