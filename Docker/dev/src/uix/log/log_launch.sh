#!/bin/bash
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 
#

# On se place en racine pour lancer Terminator
WDIR=$PWD
cd ~ 


terminator -mu -l simulation -p sim -T "Robot Simulation Station" --working-directory $WDIR
# if [***] terminator -mu -l monitoring -p monitor -T "Robot Monitoring Station" --working-directory $WDIR


# TODO : the layout would be different if we do a simulation (with the roslaunch + log the sim nodes) or a match monitoring (no roslaunch and no sim nodes -> the real actuator responses ?)
# TODO : also have different profiles ? So that we don't mistake one for the other

 # lancement d'une session de terminator avec la layout simulation
 # l'option -u permet de désactiver le dBus pour que plusieurs sessions de terminator n'interfèrent pas entre elles | TODO : empêche pyQt de fonctionner ?
 # l'option -m permet d'avoir déjà une fenêtre maximisée à l'ouverture

# kill les docker containers quand on quitte terminator
# on peut aussi utiliser stop qui arrête proprement le process mais qui met plus de temps s'il faut escalate au SIGKILL
#docker kill $(docker container ls -q)
docker stop $(docker container ls -q)


# La commande stty quit \[STOP_KEY] permet de bind temporairement une touche à la commande quit (par défaut Ctrl-\)
# Par défaut on l'utilise sur la touche "suppr" (de symbole ^[[3~ par défaut, pour avoir le symbole sur votre ordi faites dans un terminal Ctrl-V + [KEY])
# Il faut la relancer pour chaque terminal, et du coup elle ne perturbe pas les terminaux en dehors de la simulation


# On a ensuite un bind dans le bashrc qui permet de clear et de relancer en une seule touche avec bind -x '"[RESTART_KEY]":"reset && exit"'

# Et avec ça on a deux petits keybindings pour arrêter et relancer rapidement la simulation

