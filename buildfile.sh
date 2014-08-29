#!/bin/bash
#
#cleanup()
## example cleanup function
#{
#  rm -f /tmp/tempfile
#  return $?
#}

# trap keyboard interrupt (control-c)
#trap control_c SIGINT
#source setup.bash
#roscore &
#rosrun stage_ros stageros world/house.world &
#rosrun se306_project1 Main
#rosrun se306_project1 Resident &
#rosrun se306_project1 Assistant &


#trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
	killall -9 roscore
        echo "quit"
	return $?
}
source setup.bash
roscore &
rosrun stage_ros stageros world/house.world &
#rosrun se306_project1 Main
rosrun se306_project1 Resident &
rosrun se306_project1 Assistant &
rosrun se306_project1 Assistant1 &
#rosrun se306_project1 caregiver &
for i in `seq 1 5`; do
    sleep 200000
done


