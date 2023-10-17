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

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import time
import rospy

from message.msg       import InfoMsg, ActionnersMsg, EndOfActionMsg
from std_msgs.msg      import Int16, Int16MultiArray, Empty
from geometry_msgs.msg import Quaternion, Pose2D

from an_cste import *
from an_help import log_info, log_warn, log_errs

#################################################################

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################
rospy.set_param("robot_name", "PR")

def init_msgs(sm):
    """
    Init state machine in this file (no double import possible..)
    """
    global p_smach, p_udata
    p_smach = sm
    p_udata = sm.userdata

    init_pubs()
    init_subs()


def able_comm():
    """
    Set communication topics open in AN.
    """
    global ok_comm
    ok_comm = True

#################################################################
#                                                               #
#                           FEEDBACK                            #
#                                                               #
#################################################################

global ok_comm
ok_comm =False


def setup_start(msg):
	"""
    Callback function from topic /sm/start.
    """
	p_udata = (msg.data == 1)


def setup_color(msg):
    """
    Callback function from topic /sm/color.
    """
    if msg.data not in [e.value for e in ROBOT_SIDES]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    if msg.data == ROBOT_SIDES.HOME:
        log_info(f"Color received : {ROBOT_SIDES.HOME.name}")
        p_udata.color = int(ROBOT_SIDES.HOME)
    else:
        log_info(f"Color received : {ROBOT_SIDES.AWAY.name}")
        p_udata.color = int(ROBOT_SIDES.AWAY)


def recv_next_action(msg):
    """
    Callback for next action (DN -> Repartitor)
    """
    if msg.data[0] == CB_NEXT_ACTION.STOP:
        p_smach.request_preempt()
        log_info("Received stop signal (end of match)")
        return
    if msg.data[0] == CB_NEXT_ACTION.PARK:
        # Tmp fix from last year (= sm did not quit normally on parking...)
        p_smach.request_preempt()
        log_info("Received park signal (almost finished)")
        return
    if msg.data[0] not in [e.value for e in CB_NEXT_ACTION]:
        log_errs(f"Wrong command from DN [/strat/next_action] : {msg.data[0]}")
        return
    p_udata.next_act = msg.data[0]


def recv_disp_result(msg):
    """
    Callback of displacement result from Disp Node.
    """
    if not ok_comm: return
    p_udata.cb_dsp[0] = msg.data


def recv_position(msg):
    """
    Callback of current position of the robot.
    """
    if not ok_comm: return
    p_udata.cb_pos[0] = [msg.x, msg.y, msg.theta]


def recv_XXXXX(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_udata.XXX[0] = msg.data

#################################################################
#                                                               #
#                           REQUESTS                            #
#                                                               #
#################################################################

def send_added_score(pts):
    """
    Request for setting a new score (prev + new pts added).
    """
    p_udata.score[0] += pts
    score_pub.publish(p_udata.score[0])

#################################################################
#                                                               #
#                          Pubs/Subs                            #
#                                                               #
#################################################################

def init_pubs():
    """
    Initialize all publishers of AN.
    """
    # GENERAL PUBS
    global score_pub, next_action_pub, done_action_pub, next_motion_pub, stop_teensy_pub
    score_pub = rospy.Publisher('/game/score', Int16, queue_size=10, latch=True)
    next_action_pub = rospy.Publisher('/strat/next_action_requests', Empty, queue_size=10, latch=True)
    done_action_pub = rospy.Publisher('/strat/done_action', EndOfActionMsg, queue_size=10, latch=True)
    next_motion_pub = rospy.Publisher('/disp/next_displacement', Quaternion, queue_size=10, latch=True)
    stop_teensy_pub = rospy.Publisher('/stop_teensy', Quaternion, queue_size=10, latch=True)

    # SPECIFIC TO CURRENT YEAR
    # global ...
    # ...


def init_subs():
    """
    Initialize all subscribers of AN.
    """
    # GENERAL SUBS
    global start_sub, color_sub, position_sub, next_action_sub, done_motion_sub
    start_sub = rospy.Subscriber('/game/start', Int16, setup_start)
    color_sub = rospy.Subscriber('/game/color', Int16, setup_color)
    next_action_sub = rospy.Subscriber('/strat/next_action_feedback', Int16MultiArray, recv_next_action)
    done_motion_sub = rospy.Subscriber('/disp/done_displacement', Int16, ...)
    position_sub = rospy.Subscriber('/disp/current_position', Pose2D, ...)

    # SPECIFIC TO CURRENT YEAR
    # global ...
    # ...


