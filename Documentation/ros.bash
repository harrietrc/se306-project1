#!/bin/bash
roscd   
roscore
terminal -e rosrun stage_ros stageros world/house.world
terminal -e rosmake
terminal -e rosrun se306_project1 R0