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

import time
from dn_help import log_errs, log_info, log_warn, \
                    DN_LIST_ACTION_INDEX
from dn_msgs import next_action_pub, stop_IT

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

def init_strats(dn):
    global p_dn
    p_dn = dn

#################################################################
#                                                               #
#                           STRATS                              #
#                                                               #
#################################################################

def homologation_strat():
    """
    DN Strat: homologation
    
    Note: Define below the actions of this strategy 
        -
        -
        -
    """
    time.sleep(0.01)

    if p_dn.nb_actions_done[0] == 0:
        p_dn.curr_action = [int(DN_LIST_ACTION_INDEX.xxx)]
        log_info("Next action : xxx")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 1:
        p_dn.curr_action = [int(DN_LIST_ACTION_INDEX.PARK)]
        log_info("Next action : park")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 2:
        p_dn.nb_actions_done[0] =-1  # to prevent repeated end action...
        log_info("End of strategy : HOMOLOGATION")
        stop_IT()
        return


def tests_strat():
    """
    DN Strat: tests (for all tests before the cup)
    
    Note: Define below the actions of this strategy
        - 
        -
        -
    """
    time.sleep(0.01)

    if p_dn.nb_actions_done[0] == 0:
        return

    if p_dn.nb_actions_done[0] == 1:
        return


def match_strat():
    """
    DN Strat: match (only for the cup)
    
    Note: Define below the actions of this strategy
        - 
        -
        -
    """
    time.sleep(0.01)

    pass
