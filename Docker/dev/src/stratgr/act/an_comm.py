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

from std_msgs.msg      import Int16, Int16MultiArray, Empty
from geometry_msgs.msg import Quaternion, Pose2D

from an_const import *
from an_utils import log_info, log_warn, log_errs, patch_frame_br

from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg	

rospy.set_param("robot_name", "GR")

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

def init_comm(sm):
    """
    Init state machine in this file (no double import possible..)
    """
    global p_sm, p_smData
    p_sm = sm
    p_smData = sm.userdata


def enable_comm():
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
ok_comm = False


def setup_start(msg):
	"""
    Callback function from topic /sm/start.
    """
	p_smData.start = (msg.data == 1)


def setup_color(msg):
    """
    Callback function from topic /sm/color.
    """
    if not ok_comm : return
    if msg.data not in [0,1]:
        log_errs(f"Wrong value of color given ({msg.data})...")
        return
    else: 
        p_smData.color = msg.data
        log_info("Received color : {}".format(COLOR[p_smData.color]))


def cb_next_action(msg):
    """
    Callback for next action (DN -> Repartitor)
    """
    if msg.data[0] == -2:
        # Tmp fix from last year (= sm did not quit normally on parking...)
        p_sm.request_preempt()
        log_info("Received stop signal (initiate parking)")
        return
    if msg.data[0] == -1:
        p_sm.request_preempt()
        log_info("Received park signal (end of match)")
        return
    if msg.data[0] not in range(len(ACTIONS_LIST)): # Index de ACTIONS_LIST dans an_const
        log_errs(f"Wrong command from DN [/strat/repartitor] : {msg.data[0]}")
        return
    p_smData.next_action = msg.data[0]


def cb_disp(msg):
    """
    Callback of displacement result from Disp Node.
    """
    if not ok_comm: return
    p_smData.cb_disp[0] = msg.data


def cb_position(msg):
    """
    Callback of current position of the robot.
    """
    if not ok_comm: return
    p_smData.cb_pos[0] = [msg.x, msg.y, msg.theta]

def cb_arm(msg):
    """
    Callback of the state of the arm (for the cherries)
    """
    if not ok_comm: return
    p_smData.cb_arm[0] = msg.data
    
def cb_doors(msg):
    """
    Callback of the state of the doors (opened or closed) (for the cakes)
    """
    if not ok_comm: return
    p_smData.cb_doors[0] = msg.data

def cb_clamp(msg):
    """
    Callback of the state of the clamp (opened or closed) (for the cakes)
    """
    if not ok_comm: return
    p_smData.cb_clamp[0] = msg.data    

def cb_elevator(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.cb_elevator[0] = msg.data

def cb_take_cakes_area(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.take_cakes_area[0] = msg.data

def cb_take_cherries_area(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.take_cherries_area[0] = msg.data

def cb_deposit_area(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.deposit_area[0] = msg.data

def cb_stage_to_deposit(msg):
    """
    Callback of the state of the elevator (for the cakes)
    """
    if not ok_comm: return
    p_smData.stage_to_deposit[0] = msg.data

def cb_score(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.score[0] += msg.data
    score_pub.publish(p_smData.score[0])

def cb_park(msg):
    """
    Callback function to update sm variable XXXXX.

    <copy> this template for your update / callback functions.
    """
    if not ok_comm: return
    p_smData.park[0] = msg.data

#################################################################
#                                                               #
#                           ERROR                               #
#                                                               #
#################################################################

# Error action return 
def action_error(reasonOfError): 
	
	##################################
	time_out_error = 3
	##################################

	p_smData.error_reaction[0] = -1

	start_time_error = time.time()
	while p_smData.error_reaction[0] == -1 and time.time()-start_time_error < time_out_error:  # attente de reponse du DN
		time.sleep(0.05)

	if p_smData.error_reaction[0] == 1:  # On decide de faire une autre action suite a l'echec
		p_smData.error_reaction[0] = -1
		log_errs("-> REACTION : SKIP")
		return 'done'	# New action

	if p_smData.error_reaction[0] == 0:  # On decide de reessayer l'action
		p_smData.error_reaction[0] = -1
		log_errs("-> REACTION : RETRY")
		return 'redo'	# Repeat action

	if time.time()-start_time_error > time_out_error:  # Le DN n'a pas pu se decider
		log_errs("-> PAS DE REACTION DU DN")
		p_smData.error_reaction[0] = -1
		return 'done'	# Time out


# update de error_reaction
def update_error_reaction(msg):
	p_smData.error_reaction[0] = msg.data
	log_info('error_reaction mis a jour a {}'.format(p_smData.error_reaction[0]))

#################################################################
#                                                               #
#                           REQUESTS                            #
#                                                               #
#################################################################

def add_score(pts):
    """
    Request for setting a new score (prev + new pts added).
    """
    p_smData.score[0] += pts
    score_pub.publish(p_smData.score[0])

#################################################################
#                                                               #
#                          Pubs/Subs                            #
#                                                               #
#################################################################

"""
Initialize all publishers of AN.
"""
# GENERAL PUBS
global score_pub, repartitor_pub, end_of_action_pub, disp_pub, stop_teensy_pub
score_pub = rospy.Publisher('/game/score', Int16, queue_size=10, latch=True)
repartitor_pub = rospy.Publisher('/strat/repartitor_act', Empty, queue_size=10, latch=True)
end_of_action_pub = rospy.Publisher('/strat/end_of_action', EndOfActionMsg, queue_size=10, latch=True)
disp_pub = rospy.Publisher('/disp/next_displacement', Quaternion, queue_size=10, latch=True)
stop_teensy_pub = rospy.Publisher('/stop_teensy', Quaternion, queue_size=10, latch=True)

# SPECIFIC TO CURRENT YEAR
global cherries_pub, elevator_pub
cherries_pub = rospy.Publisher('/strat/cherries', Int16, queue_size=10, latch=True)
doors_pub    = rospy.Publisher('/strat/doors', Int16, queue_size=10, latch=True)
clamp_pub    = rospy.Publisher('/strat/clamp', Int16, queue_size=10, latch=True)
elevator_pub = rospy.Publisher('/strat/elevator', Int16, queue_size=10, latch=True)
pub_delete_obst = rospy.Publisher('/deleteObs', Int16, queue_size=10, latch=True)
deguis_pub = rospy.Publisher('/strat/deguisement', Int16, queue_size=10, latch=True)
force_end_pub = rospy.Publisher('/br/idle', Int16, queue_size=10, latch=True)

"""
Initialize all subscribers of AN.
"""
# GENERAL SUBS
global start_sub, color_sub, position_sub, repartitor_sub, disp_sub
start_sub = rospy.Subscriber('/game/start', Int16, setup_start)
color_sub = rospy.Subscriber('/game/color', Int16, setup_color)
repartitor_sub = rospy.Subscriber('/strat/repartitor_dec', Int16MultiArray, cb_next_action)
disp_sub = rospy.Subscriber('/disp/done_displacement', Int16, cb_disp)
position_sub = rospy.Subscriber('/current_position', Pose2D, cb_position)
park_sub = rospy.Subscriber('/park', Int16, cb_park)

# SPECIFIC TO CURRENT YEAR
global cherries_sub, elevator_sub, doors_sub, clamp_sub
cherries_sub = rospy.Subscriber('/strat/cherries_feedback', Int16, cb_arm)
doors_sub    = rospy.Subscriber('/strat/doors_feedback', Int16, cb_doors)
clamp_sub    = rospy.Subscriber('/strat/clamp_feedback', Int16, cb_clamp)
elevator_sub = rospy.Subscriber('/strat/elevator_feedback', Int16, cb_elevator)
take_cakes_sub    = rospy.Subscriber('/strat/take_cakes', Int16, cb_take_cakes_area)
take_cherries_sub    = rospy.Subscriber('/strat/take_cherries', Int16, cb_take_cherries_area)
deposit_cakes_sub = rospy.Subscriber('/strat/deposit_cakes', Int16, cb_deposit_area)
stage_sub    = rospy.Subscriber('/strat/stage', Int16, cb_stage_to_deposit)
score_sub = rospy.Subscriber('/addScore', Int16, cb_score)

