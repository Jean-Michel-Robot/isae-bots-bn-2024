#!/bin/bash


# Launch docker container
echo 'Running match'

PERIPH=$(find /dev -name ttyACM* 2>/dev/null)
echo "Périphériques connectés:"
echo $PERIPH

make main CMD="source dev/devel/setup.bash; roslaunch scripts/match.launch BR:="/dev/ttyBR" ACT:="/dev/ttyACT" LIDAR:="/dev/ttyLIDAR" NANONPX:="/dev/ttyNANONPX"; $SHELL"


#export ROS_IP=192.168.222.11
#export ROS_MASTER_URI=http://192.168.222.11:11311




#roslaunch match.launch BR:="/dev/ttyBR" ACT:="/dev/ttyACT" LIDAR:="/dev/ttyLIDAR" | tee matchLog.log
# the tee command allows to save logs in the matchLog file while keeping the display in the terminal (stdout)
