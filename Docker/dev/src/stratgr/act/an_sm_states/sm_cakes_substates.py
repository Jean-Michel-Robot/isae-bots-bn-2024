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
#																#
# 							IMPORTS 							#
#																#
#################################################################

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import smach
import time
from an_utils import log_errs, log_info, patch_frame_br

# Import publishers
from an_comm import doors_pub, clamp_pub, elevator_pub, add_score, action_error
from an_sm_states.sm_displacement import set_next_destination

# Import utils and constants
from an_const import ACTIONS_SCORE, DISPLACEMENT

#################################################################
# CONSTANTS
#################################################################

ACTION_TIMEOUT = 5
ARM_TIMING_TAKE = 3
ARM_TIMING_DEPOSE = 3

DOORS_TIMING = 2 
CLAMP_TIMING = 2
ELEVATOR_TIMING = 2

NEG_ACCURATE_DISP = -30 
POS_ACCURATE_DISP = +15

#################################################################
# MoveElevator
#################################################################

class MoveElevator(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator','nb_errors','pucks_taken','stage_to_go'],
                                    output_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator', 'nb_errors','pucks_taken','stage_to_go'])


    def execute(self, userdata):

        log_info("Ordering move elevator")
        userdata.cb_elevator[0] = -1
        elevator_pub.publish(userdata.stage_to_go[0]) # On publie 1 pour ouvrir la pince
        
        #Wait for the end of action

        """ time.sleep(6)
        userdata.nb_actions_done[0] += 1
        return 'done' """

        begin_time = time.time()
        while (time.time() - begin_time < ELEVATOR_TIMING) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_elevator[0] == 1 : #Success
                log_info("Move Elevator : success")
                #userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_elevator[0] == 0 : #Error durring the recovery
                log_info("Move Elevator : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= ELEVATOR_TIMING :
            log_info('Timeout reached')
            return action_error("fail")

#################################################################
# OpenClamp
#################################################################

class OpenClamp(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator','nb_errors','pucks_taken'],
                                    output_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator', 'nb_errors','pucks_taken'])


    def execute(self, userdata):

        log_info("Ordering open clamp")
        userdata.cb_clamp[0] = -1
        clamp_pub.publish(1) # On publie 1 pour ouvrir la pince
        
        #Wait for the end of action

        begin_time = time.time()


        time.sleep(1)
        userdata.nb_actions_done[0] += 1
        return 'done'
    
        while (time.time() - begin_time < CLAMP_TIMING) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_clamp[0] == 0 : #Success
                log_info("Open Clamp : success")
                userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_clamp[0] == 1 : #Error durring the recovery
                log_info("Open Clamp : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= CLAMP_TIMING :
            log_info('Timeout reached')
            return action_error("fail")

#################################################################
# CloseClamp
#################################################################

class CloseClamp(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator','nb_errors','pucks_taken'],
                                    output_keys=['nb_actions_done','cb_doors','cb_clamp','cb_elevator', 'nb_errors','pucks_taken'])


    def execute(self, userdata):

        log_info("Ordering close clamp")
        userdata.cb_clamp[0] = -1
        clamp_pub.publish(0) # On publie 0 pour fermer la pince
        
        #Wait for the end of action

        time.sleep(1)
        userdata.nb_actions_done[0] += 1
        return 'done'

        begin_time = time.time()
        while (time.time() - begin_time < CLAMP_TIMING) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_clamp[0] == 0 : #Success
                log_info("Close Clamp : success")
                userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_clamp[0] == 1 : #Error durring the recovery
                log_info("Close Clamp : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= CLAMP_TIMING :
            log_info('Timeout reached')
            return action_error("fail")

#################################################################
# OpenDoors - 
#################################################################

class OpenDoors(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_doors','nb_errors','pucks_taken'],
                                    output_keys=['nb_actions_done','cb_doors', 'nb_errors','pucks_taken'])


    def execute(self, userdata):

        #Deploying an arm to take cherries on a rack
        log_info("Ordering open doors")
        userdata.cb_doors[0] = -1
        doors_pub.publish(1) # On publie 1 pour dire au BN d'ouvrir les portes.
        
        #Wait for the end of action
        time.sleep(1)
        userdata.nb_actions_done[0] += 1
        return 'done'

        begin_time = time.time()
        while (time.time() - begin_time < DOORS_TIMING) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_doors[0] == 0 : #Success
                log_info("Open doors : success")
                userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_doors[0] == 1 : #Error durring the recovery
                log_info("Open doors : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= DOORS_TIMING :
            log_info('Timeout reached')
            return action_error("fail")

#################################################################
# CloseDoors - 
#################################################################

class CloseDoors(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_doors','nb_errors','pucks_taken'],
                                    output_keys=['nb_actions_done','cb_doors', 'nb_errors','pucks_taken'])


    def execute(self, userdata):

        #Deploying an arm to take cherries on a rack
        log_info("Ordering close doors")
        userdata.cb_doors[0] = -1
        doors_pub.publish(0) # On publie 0 pour dire au BN de fermer les portes.
        
        #Wait for the end of action

        time.sleep(1)
        userdata.nb_actions_done[0] += 1
        return 'done'

        begin_time = time.time()
        while (time.time() - begin_time < DOORS_TIMING) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_doors[0] == 0 : #Success
                log_info("Close doors : success")
                userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_doors[0] == 1 : #Error durring the recovery
                log_info("Close doors : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= DOORS_TIMING :
            log_info('Timeout reached')
            return action_error("fail")
