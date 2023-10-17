#!/usr/bin/env python
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
from an_sm   import init_sm
from an_msgs import init_msgs
from an_help import log_info, log_errs, log_warn


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

    log_info("Initializing ACT Node ...")
    sm = smach.StateMachine(outcomes=['EXIT_SM'])  # exit all -> exit sm
    init_sm(sm)
    init_msgs(sm)
    
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
    signal.signal(signal.SIGINT, sig_handler)
    rospy.init_node('ACT node')   
    main()