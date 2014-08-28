// Bring in my package's API, which is what I'm testing
//#include "../src/priorityQueue.h"
// Bring in gtest
//#include "../src/"
#include <gtest/gtest.h>
#include "../src/priorityQueue.cpp"

int addVars(int a, int b) {
	return a + b;
}


// Declare a test
TEST(TestSuite, testCase1)
{
	EXPECT_EQ(5, addVars(2,3));
}

TEST(TestSuite, testCase2)
{
	EXPECT_EQ(4, addVars(2,3));
}

TEST(TestSuite, testPQDummyMethod)
{
	EXPECT_EQ(1, somePQDummyTest(1,2));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
