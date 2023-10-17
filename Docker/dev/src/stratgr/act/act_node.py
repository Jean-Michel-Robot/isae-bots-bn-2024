#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import signal
import rospy
import smach
import smach_ros
import time

from an_sm import init_sm
from an_comm import init_comm
from an_utils import log_info, log_errs, log_warn


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    #############################################################
    # INITIALIZATION
    #############################################################

    signal.signal(signal.SIGINT, sig_handler)
    rospy.init_node('ACT')   

    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    log_info("Initializing Action Node ...")
    sm = smach.StateMachine(outcomes=['EXIT_SM'])  # exit all -> exit sm

    # Init SM in other files 
    ''' Les fonctions d'init sont là pour palier aux erreurs dûes aux topics.
    En gros, les topics du genre position envoient en permanence leurs infos à la comm qui modif les variables de la sm. Cependant, la comm s'initialisait avant la sm et posait des problèmes
    d'accès de varibales (car pas init dans la sm). Donc on a ces 2 fonctions init qui bloquent l'activité de leur noeud tant qu'elles ne sont pas appelées. Ainsi, on peut init la sm PUIS la comm
    sans engendrer de problèmes. 
    '''
    init_sm(sm)    # Init les variables de la sm 
    init_comm(sm)  # Init les cb et topic en ayant correctement accès aux variables
    
    #############################################################
    # SM CALL
    #############################################################
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('pr_an', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.execute()
    log_info('Exiting state machine.\n')
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
