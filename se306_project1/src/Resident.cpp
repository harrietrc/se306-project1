#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "se306_project1/ResidentMsg.h"
#include "se306_project1/DoctorMsg.h"
#include "se306_project1/AssistantMsg.h"
#include "PriorityQueue.hpp"
//#include "PriorityQueue.cpp"

#include <sstream>
#include "math.h"
#include <cmath>
#include <stdlib.h>
#include "time_conversion.hpp"
#include "Resident.h"	
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp> // Good for debugging, but take out for final build.
#include "boost/graph/breadth_first_search.hpp"

using namespace boost; // Useful for graphs

/* GRAPH STUFF */

// -- CHECKPOINTS --
// So apparently it's necessary to create an array in order to initialise a vector. Don't use this array for other 
// purposes - it's only for initialisation. Change these names (cp0, cp1, etc.) to be more descriptive - e.g. 
// kitchen, bedroom, etc. 
const char* nameArr[] = { 
	"cp0","cp1", "cp2"
};
// Vector initialised with values from the array above.
std::vector<std::string> checkpointNames(begin(nameArr), end(nameArr)); // Remember to change this - maybe equivalate to namArr length?

// -- GRAPH --
typedef property<vertex_name_t, std::string> VertexProperty; // Will allow us to retrieve vertex names from vertex references
typedef adjacency_list <vecS, vecS, undirectedS, VertexProperty> vector_graph_t; // Graph type
const int checkpointNum = checkpointNames.size(); // Number of checkpoints. 
vector_graph_t g(checkpointNum); // Our graph
std::map<std::string, vector_graph_t::vertex_descriptor> indices; // map that corresponds checkpoint names to the vertices in the graph. 

// -- EDGES --
// 'Edges' between checkpoints
typedef std::pair <std::string, std::string> E;
E paths[] = { E ("cp0", "cp1"), E ("cp1", "cp2")}; // Define edges here

// -- MAP OF NAMES TO CO-ORDINATES --
// Build a hashmap that corresponds names with checkpoint co-ordinates.
typedef std::string CheckpointName; // Key
typedef std::pair<int, int> Checkpoint; // Value
typedef std::map<CheckpointName, Checkpoint> CheckpointMap;
CheckpointMap c;
/* (end of graph stuff) */


//velocity of the robot
double linear_x;
double angular_z;

//goal pose and orientation
double goal_x;
double goal_y;
double goal_angle;
bool isSet = false;
//current pose and orientation of the robot
double px;
double py;
double cur_angle;

		int checkpoints[3][2] = {  
			{47, 43},
			{47, 20},
			{34, 20}
		};




	// Fills the property 'vertex_name_t' of the vertices, allowing us to get the checkpoint name back when
int cc = 1; //current_checkpoint = 0;
				
std::pair<double, double> move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py);
double calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py); 
void StageOdom_callback(nav_msgs::Odometry msg); 

/**
*	@brief Causes the agent to move until the goal is reached.
*	When the goal is reached, the next checkpoint becomes the goal or if the list of checkpoints is exhausted, the agent returns
*	to its initial position (the first checkpoint)
*	@param path[][2] An array of checkpoints that forms a path.
*	@param pathLength The number of checkpoints in the path (-1, as counting starts at 0)
*	@return linear_x and angular_z
*/
std::pair<double, double> Resident::movePath(int path[][2], int pathLength) {
	std::pair<double, double> ret;	
	ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	//When goal reached
	
	if ((px <= goal_x + 0.5) && (px >= goal_x - 0.5) && (py <= goal_y + 0.5) && (py >= goal_y - 0.5)) {
		isSet = false;
		if (cc == pathLength) { //If at last checkpoint
			linear_x = 0;
		} else {
			cc++; //Increment checkpoint index
		}
		if (cc == pathLength) {
			linear_x = 0;
			goal_x = path[cc-1][0];
			goal_y = path[cc-1][1];
		} else {
			goal_x = path[cc][0];
			goal_y = path[cc][1];
		}
		//Account for delay by subtracting delay values from current pose and orientation
		goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle - M_PI/20, px - 0.1, py - 0.1); 
		//goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle, px, py);

	} else { //Do this until goal is reached
		ret = move(goal_x, goal_y, cur_angle, goal_angle, px, py);	
	}
		return ret;
}


/**
*	@brief Associates checkpoint names with checkpoint co-ordinates
*	To be used in conjunction with a graph of checkpoint names, representing paths between checkpoints. Could be replaced
*	by adding the co-ordinates to bundled properties in the property map of the graph.
*/
void Resident::checkpointMap() {

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

/**
*	@brief Creates a graph of checkpoint names, provided the vector of names and the array of edges.
*	Uses boost's adjacency list.
*	@todo Error checking for non-existent edge arrays and vertex vectors. 
*/
void Resident::makeGraph() {

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
*	@brief Finds the shortest path between 2 checkpoints, and returns the path as co-ordinates.
*	Uses breadth first search - Boost recommends this over Dijkstra's algorithm for graphs with uniformly weighted edges.
*	@param startName The name of the start checkpoint as a string (e.g. 'kitchen')
*	@param endName The name of the goal checkpoint as a string (e.g. 'bathroom')
*	@return a A vector of checkpoint co-ordinates forming a path between start and end, formatted as pairs of x and y co-ordinates.
*	@todo Error checking for non-existent/initialised graph.
*/
std::vector<std::pair<double, double> > Resident::shortestPath(std::string startName, std::string endName) {

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
*	@brief Updates the Resident's x position, y position, and angle to reflect its current pose.
*	@note Rounding is used to calculate the current angle. This approximation is accounted for by using threshholds when processing angles.
*	@param msg Odometry message from odom topic
*/
void Resident::StageOdom_callback(nav_msgs::Odometry msg)
{
	/*ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	*/
	//Converting from quaternion to radians
	cur_angle = acos(msg.pose.pose.orientation.w) * 2;
	if (msg.pose.pose.orientation.z > 0) {
		cur_angle = (2*M_PI)-acos(msg.pose.pose.orientation.w) * 2;
	}

	//Rounding to 3 decimal places
	cur_angle = ((int)(cur_angle * 1000 + .5) / 1000.0);
	
	//Update the current position
	px = msg.pose.pose.position.x + checkpoints[0][0];
	py = msg.pose.pose.position.y + checkpoints[0][1];

}

/**
*	@brief Callback function to process laser scan messsages.
*	You can access the range data from msg.ranges[i]. i = sample number
*	@note Currently blank as it is not in use. Navigation operates through a checkpoint system.
*	@param msg Single scan from a planar laser range finder
*/
void Resident::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

/**
*	@brief Increases the resident's health when the doctor heals them.
*	@param msg A custom message from an Assistant robot.
*	@remarks Perhaps add a delay between medication/diagnosis and healing?
*/
void Resident::doctor_callback(se306_project1::DoctorMsg msg)
{

	 if (msg.healResident == 1)
	{
	 	health = 100;
		ROS_INFO("Resident healed by Doctor, health = 100");
	}

}

/**
*	@brief Increases the Resident's hunger (towards full) if food has been delivered.
*	@param msg A custom message from an Assistant robot. 
*/
void Resident::assistant_callback(se306_project1::AssistantMsg msg)
{
	if (msg.FoodDelivered == 1)
	{
		hunger = 100;
		ROS_INFO("Resident has received food");
	}
	
}

/**
*	@brief Keeps the agent moving by changing linear_x ad angular_z.
*	@param goal_x The x position of the robot's goal
*	@param goal_y The y position of the robot's goal
* 	@param cur_angle The agent's current facing, in reference to the co-ordinate system.
*	@param goal_angle The angle that the agent must face in order to reach the goal.
*	@param px Initial x position
*	@param py Initial y position
*	@return _ret linear_x and angular_z
*/
std::pair<double, double> Resident::move(double goal_x, double goal_y, double cur_angle, double goal_angle, double px, double py) 
{	
	std::pair<double,double>_ret = std::make_pair(0, 0); //initialize pair. Used to get return.
	double moveSpeed = M_PI/2;
	moveSpeed = ((int)(moveSpeed * 1000 + .5) / 1000.0);

	//When the robot is facing the correct direction, start moving
	double threshold = cur_angle;//-moveSpeed/10;
	//threshold = ((int)(threshold * 1000 + .5) / 1000.0);

	if ((goal_angle  == threshold) || isSet) {
		_ret.first = 5; //linear_x
		_ret.second = 0; //angular_z
		isSet = true;
	} else if ((goal_angle <= cur_angle + 0.6) && (goal_angle >= cur_angle - 0.6) )  {
		_ret.first = 0; //linear_x
		_ret.second = fabs(goal_angle - cur_angle); //angular_z
		if (goal_angle == cur_angle) {
			isSet = true;		
		}

	} else {
		_ret.first = 0; //linear_x
		_ret.second = moveSpeed; //angular_z
		isSet = false;
	}


	if ((px-0.1 <= goal_x + 0.5) && (px-0.1 >= goal_x - 0.5) && (py-0.1 <= goal_y + 0.5) && (py-0.1 >= goal_y - 0.5)) {	
			_ret.first = 0; 
			isSet = false;
	}

	return _ret; 
}

/**
*	@brief Given the agent's current angle, this function calculates the angle to the goal.
*	cur_angle, goal_x, goal_y, px, and py are class fields but are also passed as parameters.
*	@param goal_x The x co-ordinate of the goal
*	@param goal_y The y co-ordinate of the goal
*	@param cur_angle The agent's current angle, in reference to the co-ordinate system
*	@param px The agent's initial x position
*	@param py The agent's initial y position
*	@param goal_angle The angle that the robot must rotate to face the goal, in reference to the co-ordinate system.
*/
double Resident::calc_goal_angle(double goal_x, double goal_y, double cur_angle, double px, double py) 
{

	//Initial and goal vectors used to calculate goal theta
	double init_vector_x;
	double init_vector_y;
	double goal_vector_x;
	double goal_vector_y;
	double goal_angle;
	
	//Finding the vector that the robot is facing and the goal vector
	init_vector_x = cos(cur_angle);
	init_vector_y = sin(cur_angle);
	goal_vector_x = goal_x - px;
	goal_vector_y = goal_y - py;
	
	goal_angle = atan2(goal_vector_y, goal_vector_x); //pi <= goal_angle < -pi
	if (goal_angle < 0) {
		goal_angle = 2 * M_PI + goal_angle; //Remove sign, then add to pi
	} else if (goal_angle == 2 * M_PI) { //New goal angle =>   >0 to 6.283
		goal_angle = 0;
	}
	goal_angle = (2* M_PI) - goal_angle;

	//rounding goal_angle to three decimal places
	goal_angle = ((int)(goal_angle * 1000 + .5) / 1000.0);
	
	return goal_angle;
}

/*
	Simple model of resident - resident should move to a random checkpoint every once in a while. Will be replaced with a
	more complex and realistic model later.
*/
void Resident::randomCheckpointCallback(const ros::TimerEvent&) {
	//ROS_INFO("hello");
}

/**
*	@brief Main function for the Resident process.
*	Controls node setup and periodic events.
*/
int Resident::run(int argc, char *argv[]) {

	// Graph demo (pass start and goal checkpoint names, get a path of checkpoints as co-ordinates back.)
	checkpointMap();
	makeGraph();
	std::vector<std::pair<double, double> > sp = shortestPath("cp0", "cp2"); // Gets shortest path from cp0 to cp1
	for (int i=0; i<sp.size(); i++) {
		printf("%f, %f\n", sp[i].first, sp[i].second); 
	} // Note that it won't return the start node - if you want it to, there's a line you can uncomment in shortestPath().

	//Initial pose. This is the same as the pose used in the world file.
	px = checkpoints[cc-1][0];
	py = checkpoints[cc-1][1];
	cur_angle = 0;

	//Set goal pose 
	goal_x = checkpoints[cc][0];
	goal_y = checkpoints[cc][1];

	goal_angle = calc_goal_angle(goal_x, goal_y, cur_angle, px, py);

	//Initial velocities
	linear_x = 0;
	angular_z = 0;
	
	//Align local system to global coordinates

	health = 100;
	boredom = 100;
	hunger = 100;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Resident");
	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000); 

	//custom message/topic publisher "resident/state" for now
	//ros::Publisher Resident_pub = n.advertise<std_msgs::String>("residentStatus",1000); 
	ros::Publisher resident_pub = n.advertise<se306_project1::ResidentMsg>("residentStatus",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, &Resident::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,&Resident::StageLaser_callback, this);

	//subscribe to doctor messages
	ros::Subscriber doctor_sub = n.subscribe<se306_project1::DoctorMsg>("healResident",1000, &Resident::doctor_callback, this);

	ros::Subscriber assistant_sub = n.subscribe<se306_project1::AssistantMsg>("assistantStatus",1000, &Resident::assistant_callback, this);

	//ros::Rate loop_rate(10);
	ros::Rate loop_rate(1000);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	// Periodic callback
	int dur2 = time_conversion::simHoursToRealSecs(2); // Perform callback every 2 simulation hours
	ros::Timer medicationTimer = n.createTimer(ros::Duration(dur2), &Resident::randomCheckpointCallback, this); 

	int hungerReductionRate = 2; //1 hunger point reduction per second
	int healthReductionRate = 2; // 0.1 health point reduction per second
	
	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		//ROS_INFO("Hunger %d",hunger);
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		// Reduces hunger every second
		if (count % 100 == 0){
				hunger -= hungerReductionRate;
				health -= healthReductionRate;
		}
			std::pair<double, double> velocityValues;	
			velocityValues = std::make_pair(0, 0);
		if (hunger < 90) {
			velocityValues = movePath(checkpoints, 	3);
			linear_x = velocityValues.first;
			angular_z = velocityValues.second;			
		}
		
		se306_project1::ResidentMsg msg; 
		msg.health = health;
		msg.hunger = hunger;
		msg.x = px; 
		msg.y = py;

		resident_pub.publish(msg);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}

/**
*	@brief Redirects to main function (run()) of the node.
*/
int main(int argc, char **argv) {
	Resident *a = new Resident();
	a->Resident::run(argc, argv);
}
