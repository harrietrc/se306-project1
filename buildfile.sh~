#!/bin/bash

source setup.bash
rosmake se306_project1
roscore &
rosrun stage_ros stageros world/house.world &
rosrun se306_project1 Assistant

