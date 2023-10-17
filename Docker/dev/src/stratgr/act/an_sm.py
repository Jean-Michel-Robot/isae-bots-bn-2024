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

# import les states de la SM
from an_sm_states.sm_park import Park
from an_sm_states.sm_waiting import Waiting
from an_sm_states.sm_cherries_take_perpendicular import TakeCherriesPerpendicular
from an_sm_states.sm_cherries_take_wall import TakeCherriesWall
from an_sm_states.sm_cherries_deposit import DepositCherries
from an_sm_states.sm_cakes_take import TakeCakes
from an_sm_states.sm_cakes_deposit import DepositCakes
from an_sm_states.sm_cherries_deposit_near import DepositCherriesNear
from an_sm_states.sm_push_cakes import PushCakes

from an_const import *
from an_utils import *
from an_comm  import enable_comm, repartitor_pub, disp_pub, stop_teensy_pub

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class Setup(smach.State):
	"""
    STATE MACHINE: setup the SM.
    """

	def __init__(self):
		smach.State.__init__(	self, 	
								outcomes=['start', 'preempted'],
								input_keys=ALL_KEY_LIST,
								output_keys=ALL_KEY_LIST)

	def execute(self, userdata):
		##############################
		## VARIABLES INITIALIZATION ##
		##############################
		
		## Game param variables
		userdata.start = False
		userdata.color = 0
		userdata.score = [ACTIONS_SCORE['init_score']]
		userdata.nb_actions_done = [0]
		userdata.park = [0] 
		
		## Data about the match
		userdata.deposit_area = [-1] # Coordonnées de là où on dépose les gâteaux
		userdata.take_cakes_area = [-1] 	# Coordonnées de la pile de gâteaux qui nous intéressent.
		userdata.take_cherries_area = [-1] 	# Coordonnées du rack de cerises qui nous intéressent.
		userdata.pucks_taken = [0]	# Pemet de savoir combien on transporte de palets pour pouvoir savoir comment on récupère les autres
		userdata.cherries_loaded = [0] # Permet de savoir si le robot transporte des cerises ou non. 0: Non ; 1: Oui 
		userdata.stage_to_go = [0]
		userdata.stage_to_deposit = [0]

		## Callback of subscribers
		userdata.cb_disp = [-1]  # result of displacement action. CHECK an_const to see details on cb_disp
		userdata.cb_pos = [[]]  # current position of the robot
		userdata.cb_arm = [-1]    # state of the arm
		userdata.cb_doors = [-1]	# state of the doors
		userdata.cb_clamp = [-1] 	# state of the clamp
		userdata.cb_elevator = [-1] # state of the elevator

		## Game infos variables
		userdata.next_action = -2  # Indicateur de l'action en cours
		userdata.next_pos = Quaternion(x=0, y=0, z=0, w=1)
		userdata.error_reaction = [-1]
		userdata.nb_errors = [0]
		userdata.backward = False
		userdata.open_clamp = False
		userdata.open_doors = False 
		userdata.elevator_zero = True

		## Enable pubs and subs in pr_an_comm.py
		time.sleep(0.01)
		enable_comm()

		##############################
		## WAITING FOR START SIGNAL ##
		##############################
		log_info('Waiting for START signal ...')

		while not userdata.start:
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)
		
		log_info('Starting match !')
		return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class Repartitor(smach.State):
	"""
    STATE MACHINE : Dispatch actions between sm substates.
    """

	def __init__(self):
		smach.State.__init__(	self, 	
		       					outcomes=ACTIONS_LIST,
								input_keys=['nb_actions_done', 'next_action', 'pucks_taken'],
								output_keys=['nb_actions_done', 'next_action'])

	def execute(self, userdata):
		log_info('[Repartitor] Requesting next action ...')
		log_info('[PUCKS ACTUALLY TAKEN] = ' + str(userdata.pucks_taken[0]))
		repartitor_pub.publish(Empty()) 	         # demande nextAction au DN

		userdata.next_action = -2 				   # reset variable prochaine action
		while userdata.next_action == -2:  # en attente de reponse du DN
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			time.sleep(0.01)

		userdata.nb_actions_done[0] = 0  		     	  # reinitialisation nb etapes
		return 	ACTIONS_LIST[userdata.next_action]   # lancement prochaine action  
		
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self):
        smach.State.__init__(	self, 	
			     				outcomes=['end','preempted'],
            					input_keys=[''],
            					output_keys=[''])

    def execute(self, userdata):
        log_info('[End] Killing state machine ...')

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        disp_pub.publish(Quaternion(x=0,y=0,z=0,w=-1))			# arrêt PF : w = -1
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
		### Primary States
		smach.StateMachine.add('SETUP', 
			 					Setup(),
								transitions={'preempted':'END','start':'REPARTITOR'})
		smach.StateMachine.add('REPARTITOR', 
			 					Repartitor(),
								transitions=ACTIONS_STATES)
		smach.StateMachine.add('END', 
			 					End(),
								transitions={'end':'EXIT_SM','preempted':'EXIT_SM'})

		### Secondary States
		smach.StateMachine.add('TAKE_CHERRIES_PERPENDICULAR', 
			 					TakeCherriesPerpendicular,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('TAKE_CHERRIES_WALL', 
			 					TakeCherriesWall,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('DEPOSIT_CHERRIES', 
			 					DepositCherries,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('DEPOSIT_CHERRIES_NEAR', 
			 					DepositCherriesNear,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('TAKE_CAKES', 
			 					TakeCakes,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('DEPOSIT_CAKES', 
			 					DepositCakes,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('PARK', 
			 					Park,
								transitions={'preempted':'END','end':'END'})
		smach.StateMachine.add('WAITING', 
			 					Waiting,
								transitions={'preempted':'END','end':'REPARTITOR'})
		smach.StateMachine.add('PUSH_CAKES', 
			 					PushCakes,
								transitions={'preempted':'END','end':'REPARTITOR'})
