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
import rospy
import signal

from dn_help   import CONFIG_READER, log_errs, log_info, log_warn, \
                      STRAT_NAMES, DN_LIST_ACTION_NAMES
from dn_msgs   import init_msgs
from dn_strats import init_strats, \
                      homologation_strat, tests_strat, match_strat

#################################################################
if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg  # sur robot
else:
	from message.msg            import InfoMsg, ActionnersMsg, EndOfActionMsg  # sur ordi
#################################################################

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

        self.match_strated = False
        self.color = 0
        self.strat = int(CONFIG_READER.get("Strat", "strat_choice"))
        self.strategies = [e.values for e in STRAT_NAMES]

        self.start_time = 0
        self.match_time = int(CONFIG_READER.get("Strat", "match_time"))
        self.delay_park = 13  # TODO: change it to named constant

        self.actions_ls = [e.values for e in DN_LIST_ACTION_NAMES]
        self.actions_nb = len(self.actions_ls)
        self.curr_action = []
        self.nb_actions_done = [0]

        self.position = [0,0,0]

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

def main():
    rospy.init_node("DEC node")
    node = DecisionsNode()

    init_msgs(node)
    init_strats(node)

    rospy.spin()


if __name__ == "__main__":
    main()