cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(Resident src/Resident.cpp src/time_conversion.cpp src/Agent.cpp src/CheckPointGraph.cpp src/priorityQueue.cpp)
rosbuild_add_executable(Assistant src/Assistant.cpp src/time_conversion.cpp src/Agent.cpp src/CheckPointGraph.cpp)
rosbuild_add_executable(Assistant1 src/Assistant1.cpp src/time_conversion.cpp src/Agent.cpp src/CheckPointGraph.cpp)

rosbuild_add_executable(Doctor src/Doctor.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp)
rosbuild_add_executable(Caregiver src/Caregiver.cpp src/Agent.cpp src/CheckPointGraph.cpp)
rosbuild_add_executable(Friend src/Friend.cpp src/Agent.cpp src/Visitor.cpp src/time_conversion.cpp src/CheckPointGraph.cpp)
rosbuild_add_executable(Nurse src/Nurse.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp)
#rosbuild_add_executable(Relative src/Relative.cpp src/Agent.cpp src/CheckPointGraph.cpp)
rosbuild_add_executable(Door src/Door.cpp)


#rosbuild_add_executable(R0 src/R0.cpp src/time_conversion.cpp)
#rosbuild_add_executable(R0 src/R0.cpp)

#rosbuild_add_executable(R0 src/R0.cpp)
rosbuild_add_executable(Main src/Main.cpp src/AgentFactory.cpp src/ProcessManager.cpp)
#rosbuild_add_executable(R1 src/R1.cpp)

rosbuild_add_executable(Friend1 src/Friend1.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Friend2 src/Friend2.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Friend3 src/Friend3.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Nurse1 src/Nurse1.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Nurse2 src/Nurse2.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Caregiver1 src/Caregiver1.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
rosbuild_add_executable(Caregiver2 src/Caregiver2.cpp src/Agent.cpp src/Visitor.cpp src/CheckPointGraph.cpp src/time_conversion.cpp )
