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
import rospy
import signal
import time

from dn_utils    import READER, ACTIONS_LIST, log_errs, log_fatal, log_info, log_warn, ACTIONS_SCORE
from dn_comm     import init_comm
from dn_strats   import init_strats, test_strat, homologation, match_strat

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()

#################################################################
#                                                               #
#                           DEC node                            #
#                                                               #
#################################################################

class DecisionsNode:
    """
    DEC node: ROS node for decisions and strategy.
    """

    def __init__(self):

        log_info("Initializing DEC node ...")

        self.match_started = False
        self.color = 0
        self.score = ACTIONS_SCORE['init_score']

        self.start_time = 0
        self.match_time = int(READER.get("STRAT", "match_time"))
        self.delay_park = 13  # TODO: change it to named constant
        self.go_park = False

        self.strat = int(READER.get("STRAT", "strat_choice"))
        self.strategies = [match_strat, homologation, test_strat]

        self.actions_ls = ACTIONS_LIST
        self.actions_nb = len(self.actions_ls)
        self.park_action = False
        self.kill_action = False
        self.curr_action = []
        self.nb_actions_done = [0]

        self.position = [0,0,0]

#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rospy.init_node("DEC")

    time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

    node = DecisionsNode()

    init_comm(node)
    init_strats(node)

    log_info("Waiting for match to start")

    rospy.spin()


if __name__ == "__main__":
    main()