#!/bin/bash


# export ROS_IP=192.168.43.12
# export ROS_MASTER_URI=http://192.168.43.12:11311

#rostopic echo -p --nostr --filter="m.name=='$1'" /rosout_agg/msg | grep -v % | bash -c "echo $@"
#echo "1660565532214021205,0,1660565532214021205,,8,/ACT,The test error,an_utils.py,log_info,36,/pr_an/smach/container_init,/strat/next_action_request,/disp/current_position,/strat/done_action,/game/score,/pr_an/smach/container_status,/rosout,/game/start,/stop_teensy,/game/color,/disp/done_displacement,/strat/next_action_answer,/disp/next_displacement,/pr_an/smach/container_structure" | awk -F, '{printf "First one %s and second one %s\n", $4, $5}'
rostopic echo -p --filter="m.name=='$1'" /rosout_agg/msg | grep -v %

# TODO : enlever les \ qui sont mis pour les double quotes dans le rosout_agg