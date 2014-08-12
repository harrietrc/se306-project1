#!/bin/bash

source setup.bash
./se306_project1/rosmake
roscore &
rosrun stage_ros stageros world/house.world &
rosrun se306_project1 R0

