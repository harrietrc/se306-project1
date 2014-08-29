// Bring in my package's API, which is what I'm testing
#include "../src/priorityQueue.cpp"
#include "../src/priorityQueue.h"
#include "../src/CheckPointGraph.cpp"
//#include "../src/CheckPointGraph.hpp"
// Bring in gtest
//#include "../src/"
#include <gtest/gtest.h>

/*Tests methods when pq is initialised*/
TEST(PqTestSuite, emptyPqTest)
{
	//Empty Priority queue
	priorityQueue *a = new priorityQueue();
	//Priority queue initially set to tired as the resident is sleeping when we start the simulation.
	EXPECT_EQ("tired", a->priorityQueue::checkCurrentState());
	EXPECT_EQ(tired, a->priorityQueue::popFromPQ());

	//No more states in PQ so returns idle
	EXPECT_EQ(idle, a->priorityQueue::popFromPQ());
	//PQ is still empty so still returns idle
	EXPECT_EQ(idle, a->priorityQueue::popFromPQ());
	
}

/*Tests adding and popping the queue*/
TEST(PqTestSuite, semiFilledPqTest)
{
	//Priority queue containing 2 states
	priorityQueue *a = new priorityQueue();
  a->priorityQueue::addToPQ(hungry);
	a->priorityQueue::addToPQ(caregiver);

	//EXPECT_EQ("caregiver", a->priorityQueue::stateConvertString(caregiver));
	EXPECT_EQ("caregiver", a->priorityQueue::checkCurrentState());
	EXPECT_EQ(caregiver, a->priorityQueue::popFromPQ());

	EXPECT_EQ("hungry", a->priorityQueue::checkCurrentState());
	EXPECT_EQ(hungry, a->priorityQueue::popFromPQ());
 
	EXPECT_EQ("tired", a->priorityQueue::checkCurrentState());
	EXPECT_EQ(tired, a->priorityQueue::popFromPQ());

	EXPECT_EQ("idle", a->priorityQueue::checkCurrentState());
}

/*Test order of priorities  a full PQ*/
TEST(PqTestSuite, fullPqTest)
{
	//Priority Queue containing all the states
	priorityQueue *a = new priorityQueue();
  a->priorityQueue::addToPQ(bored);
	a->priorityQueue::addToPQ(tired);
	a->priorityQueue::addToPQ(hungry);
	a->priorityQueue::addToPQ(friends);
	a->priorityQueue::addToPQ(medication);
	a->priorityQueue::addToPQ(caregiver);
	a->priorityQueue::addToPQ(healthLow);
	a->priorityQueue::addToPQ(emergency);
	a->priorityQueue::addToPQ(emergency); //pushed emergency twice to check behaviour but when popped will return once.

	EXPECT_EQ("emergency", a->priorityQueue::checkCurrentState());
	EXPECT_EQ(emergency, a->priorityQueue::popFromPQ());
	
	//health low should not exist simultaneously with emergency. The next highest states is caregiver.
	EXPECT_EQ("caregiver", a->priorityQueue::checkCurrentState()); 
	EXPECT_EQ(caregiver, a->priorityQueue::popFromPQ());

}
//======================================================================//

/*Testing the CheckPointGraph class*/
TEST(GraphTestSuite, simpleCase)
{
	CheckPointGraph *g = new CheckPointGraph();
	std::vector<std::pair<double, double> > testPath;
	testPath = g->CheckPointGraph::shortestPath("KitchenNorthWest", "HouseCentre");
	EXPECT_EQ(checkpoints[6][0], testPath.at(0).first);
	EXPECT_EQ(checkpoints[6][1], testPath.at(0).second);

	EXPECT_EQ(checkpoints[10][0], testPath.back().first);
	EXPECT_EQ(checkpoints[10][1], testPath.back().second);
	//EXPECT_TRUE(true);
	
}

/*Checks if the shortest path is taken when given checkpoints that are far apart*/
TEST(GraphTestSuite, longPathCase)
{
	CheckPointGraph *g = new CheckPointGraph();
	std::vector<std::pair<double, double> > testPath;
	testPath = g->CheckPointGraph::shortestPath("KitchenSouthEast", "shower");
	EXPECT_EQ(6, testPath.size());
	
}


/*Checks if the shortest path is taken when given checkpoints that are far apart*/
TEST(GraphTestSuite, checkpointNameTest)
{
	CheckPointGraph *g = new CheckPointGraph();
	std::pair<double, double> testPair = std::make_pair(-27, -40);
	EXPECT_EQ("FrontDoorWest", g->CheckPointGraph::getCheckpointName(testPair));
	
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}

