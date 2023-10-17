#!/bin/bash

source /opt/ros/noetic/setup.bash

sleep 2
if [ -z $(rosnode list) ]; then ROSLAUNCH_RUNNING=0; else ROSLAUNCH_RUNNING=1; fi
clear

# BINDKEYS

COMMANDK="[SIM COMMAND] :  Shutting down roslaunch..."
COMMANDK1="[SIM COMMAND] -> stopped roslaunch"
COMMANDK2="\033[31m[ERROR] No roslaunch is currently active\033[0m"
COM="ps -ef | grep /bin/roslaunch | awk 'NR==1{print \$2}'"
bind -x '"\C-K":"if [[ $ROSLAUNCH_RUNNING == 1 ]]; then echo $COMMANDK; ROSLAUNCH_RUNNING=0; kill -s SIGINT $(eval $COM); sleep 2; echo $COMMANDK1; else echo -e $COMMANDK2; fi"'
# pour interrompre (mais pas exit) le roslaunch, on envoie un SIGINT (Ctrl-C) au roslaunch, pour avoir son PID on parse le résultat de la commande ps

COMMANDR="[SIM COMMAND] :  Restart simulation"
COMMANDR2="\033[31m[ERROR] Stop the roslaunch before restarting\033[0m"
COM2="ps -ef | grep /bin/bash | grep -vE 'exit|auto|grep|dev' | awk '{print \$2}'"
bind -x '"\C-R":"if [[ $ROSLAUNCH_RUNNING == 0 ]]; then echo $COMMANDR; ROSLAUNCH_RUNNING=1; kill $(eval $COM2); else echo -e $COMMANDR2; fi"'
# pour exit le roslaunch (ce qui ferme le main du docker et donc relance tous les terminaux docker), on envoie un SIGHUP au roslaunch (qui a été config pour exit à réception d'un 
# SIGHUP via la commande trap), pour avoir son PID on parse le résultat de la commande ps aussi

COMMANDH="[ROS MESSAGE] -> Set side to \033[36mHOME\033[0m"
bind -x '"\C-H":"echo -e $COMMANDH; rostopic pub --once /game/color std_msgs/Int16 0 > /dev/null"'

COMMANDA="[ROS MESSAGE] -> Set side to \033[36mAWAY\033[0m"
bind -x '"\C-A":"echo -e $COMMANDA; rostopic pub --once /game/color std_msgs/Int16 1 > /dev/null"'

COMMANDG="[ROS MESSAGE] -> \033[32mStart match\033[0m"
bind -x '"\C-G":"echo -e $COMMANDG; rostopic pub --once /game/start std_msgs/Int16 1 > /dev/null"'


# INFO MESSAGE AT STARTUP

cols="$(tput cols)"
((cols=cols/2+13))
printf "%*s\n" $cols "#########################"
printf "%*s\n" $cols "#  Simulation Terminal  #"
printf "%*s\n" $cols "#########################"

echo -e "\033[1mKey Commands :\033[0m\n\
Ctrl-H : set side to HOME\n\
Ctrl-A : set side to AWAY\n\
Ctrl-G : start match\n\
\n\
Ctrl-D : clear terminal\n\
Ctrl-K : stop roslaunch\n\
Ctrl-R : restart the simulation\n\
Ctrl-Q : exit the simulation\n"