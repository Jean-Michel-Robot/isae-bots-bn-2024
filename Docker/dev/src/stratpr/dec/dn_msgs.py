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
import time
import rospy
import threading
from dn_help import log_info, log_errs, log_warn, \
                    ROBOT_SIDES, STRAT_NAMES, STRAT_INDEX, CB_NEXT_ACTION, \
                    TERM_SIZE
from std_msgs.msg      import Empty, Int16, Int16MultiArray
from geometry_msgs.msg import Quaternion, Pose2D

#################################################################
if os.environ['USER'] == 'pi':
    from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg
else:
    from message.msg            import InfoMsg, ActionnersMsg, EndOfActionMsg
#################################################################

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

def init_msgs(dn):
    """
    Init the decision node in this file (no double imports)
    """
    global p_dn
    p_dn = dn

#################################################################
#                                                               #
#                           FEEDBACK                            #
#                                                               #
#################################################################

def start_match(msg):
    """
    Feedback on start signal /game/start
    """
    if p_dn.match_started: return

    if msg.data == 1:
        log_info("#"*TERM_SIZE + "\n"
                 "#"*4 + " Start match " + "#"*(TERM_SIZE-17) + "\n"
                 "#"*TERM_SIZE)

        p_dn.match_started = True
        p_dn.start_time = time.time()
        threading.Timer(p_dn.match_time - p_dn.delay_park, park_IT).start()
        threading.Timer(p_dn.match_time, stop_IT).start()


def setup_color(msg):
    """
    Feedback on color side /game/color.
    """
    p_dn.color = msg.data
    if p_dn.color == ROBOT_SIDES.HOME:
        log_info("Received color : HOME")
    else:
        log_info("Received color : AWAY")


def setup_strat(msg):
    """
    Feedback on strategy chosen /game/strat.*
    """
    if msg.data == STRAT_INDEX.HOMOLOGATION:
        p_dn.strat_index = STRAT_INDEX.HOMOLOGATION
        log_info(f"Received strat: {STRAT_NAMES.HOMOLOGATION}")
        return
    if msg.data == STRAT_INDEX.TESTS:
        p_dn.strat_index = STRAT_INDEX.TESTS
        log_info(f"Received strat: {STRAT_NAMES.TESTS}")
        return
    if msg.data == STRAT_INDEX.MATCH:
        p_dn.strat_index = STRAT_INDEX.MATCH
        log_info(f"Received strat: {STRAT_NAMES.MATCH}")
        return
    log_errs(f"Wrong strat index received: {msg.data}")


def recv_position(msg):
    """
    Feedback on /disp/current_position topic.
    """
    p_dn.position = [msg.x, msg.y, msg.theta]


def recv_action_done(msg):
    if msg.exit == 1:
        log_info("Last action succeeded.")
        p_dn.nb_actions_done[0] += 1
        return

    if msg.exit ==-1:
        log_errs(f"Last action failed, reason: {msg.reason}")
        return
    log_errs("Wrong value sent on /strat/done_action ...")


def send_action_next(msg):
    """
    Send back the next action when triggered.
    """
    log_info(f"Next action requested by AN ...")
    p_dn.strat_lst[p_dn.strat]()
    

#################################################################
#                                                               #
#                         INTERRUPTION                          #
#                                                               #
#################################################################

def park_IT():
    """
    Interrupt : time to park
    """
    log_info("#"*TERM_SIZE + "\n"
             "#"*4 + " Park interrupt " + "#"*(TERM_SIZE-20) + "\n"
             "#"*TERM_SIZE)
    next_action_pub.publish(data=[CB_NEXT_ACTION.PARK_IT])


def stop_IT():
    """
    Interrupt : end of match
    """
    log_info("#"*TERM_SIZE + "\n"
             "#"*4 + " End of match " + "#"*(TERM_SIZE-18) + "\n"
             "#"*TERM_SIZE)
    next_action_pub.publish(data=[CB_NEXT_ACTION.STOP_IT])

#################################################################
#                                                               #
#                           Pubs/Subs                           #
#                                                               #
#################################################################

def init_pubs():
    global next_action_pub
    next_action_pub = rospy.Publisher("/strat/next_action_feedback", Int16MultiArray, queue_size=10, latch=True)


def init_subs():
    global start_sub, color_sub, strat_sub, position_sub
    start_sub = rospy.Subscriber("/game/start", Int16, start_match)
    color_sub = rospy.Subscriber("/game/color", Int16, setup_color)
    strat_sub = rospy.Subscriber("/game/strat", Int16, setup_strat)
    position_sub = rospy.Subscriber("/disp/current_position", Pose2D, recv_position)

    global next_action_sub
    next_action_sub = rospy.Subscriber("/strat/next_action_requests", Empty, send_action_next)
    done_action_sub = rospy.Subscriber("/strat/done_action", EndOfActionMsg, recv_action_done)