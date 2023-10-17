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

import time
import smach

from std_msgs.msg      import Empty
from geometry_msgs.msg import Quaternion

# import SM states defined in an_sm_states package
from an_sm_states.sm_park import SM_Park
from an_sm_states.sm_move import SM_Move

from an_cste import *
from an_help import log_info, log_warn, log_errs
from an_msgs import able_comm, next_action_pub, next_motion_pub, \
                    stop_teensy_pub

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class SM_Setup(smach.State):
	"""
    STATE MACHINE: setup the SM.
    """

	def __init__(self):
		smach.State.__init__(self, outcomes=['start', 'preempted'],
			input_keys=[],
			output_keys=[])

	def execute(self, userdata):
		##############################
		## VARIABLES INITIALIZATION ##
		##############################
		
		## Game param variables
		userdata.start = False
		userdata.color = 0
		userdata.score = 0
		userdata.nb_actions_done = [0]
		
		## Callback of subscribers
		userdata.cb_dsp = [-1]  # result of displacement action
		userdata.cb_pos = [[]]  # current position of the robot

		

		userdata.next_act = -2  # Indicateur de l'action en cours
		userdata.next_pos = Quaternion(x=0, y=0, z=0, w=1)
		userdata.errorReaction = [-1]
		userdata.errorActions = [0]

		## Enable pubs and subs in pr_an_comm.py
		time.sleep(0.01)
		able_comm()

		##############################
		## WAITING FOR START SIGNAL ##
		##############################
		log_info('[smach] waiting for "start" signal ...')

		while not userdata.start:
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)
		
		log_info('[smach] starting match !')
		return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class SM_Repartitor(smach.State):
	"""
    STATE MACHINE : Dispatch actions between sm substates.
    """

	def __init__(self):
		smach.State.__init__(self, outcomes=[],
			input_keys=[],
			output_keys=[])

	def execute(self, userdata):
		log_info('[repartitor] requesting next action ...')
		next_action_pub.publish(Empty()) 	         # demande nextAction au DN

		userdata.next_act = CB_NEXT_ACTION.NONE	     # reset variable prochaine action
		while userdata.next == CB_NEXT_ACTION.NONE:  # en attente de reponse du DN
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)

		userdata.nb_actions_done[0] = 0  		     # reinitialisation nb etapes
		return None	                                 # lancement prochaine action  
		
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class SM_End(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['end','preempted'],
            input_keys=[],
            output_keys=[])

    def execute(self, userdata):
        log_info('[end] killing state machine ...')

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        next_motion_pub.publish(Quaternion(x=0,y=0,z=0,w=-1))	# arrêt PF : w = -1
        stop_teensy_pub.publish(Quaternion(x=0,y=0,z=0,w=2))	# arrêt BR (code w=2 pour le BN)
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

def init_sm(sm):
	"""
	Init state machine with its substates.
	"""

	with sm:
		smach.StateMachine.add('SETUP', SM_Setup(),
			transitions={'preempted':'END','start':'REPARTITOR'})
		smach.StateMachine.add('REPARTITOR', SM_Repartitor(),
			transitions={})
		smach.StateMachine.add('END', SM_End(),
			transitions={'end':'EXIT_SM','preempted':'EXIT_SM'})

		smach.StateMachine.add('PARK', SM_Park,
			transitions={'preempted':'REPARTITOR','end':'REPARTITOR'})
		smach.StateMachine.add('MOVE', SM_Move,
			transitions={'preempted':'REPARTITOR','end':'REPARTITOR'})
