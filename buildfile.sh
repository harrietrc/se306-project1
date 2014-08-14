#!/bin/bash

source setup.bash
roscore &
rosrun stage_ros stageros world/house.world &
rosrun se306_project1 Resident &
rosrun se306_project1 Assistant &

