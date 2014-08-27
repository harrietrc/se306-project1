// Bring in my package's API, which is what I'm testing (Doctor for this test)
#include "../src/Doctor.cpp"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(DoctorTest, goalAngle)
{
	Doctor *a = new Doctor;
	EXPECT_EQ(10, a->Doctor::calc_goal_angle(1, 1, 1, 1, 1));
}



