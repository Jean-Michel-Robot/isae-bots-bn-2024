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
from an_comm import cherries_pub, add_score, action_error
from an_sm_states.sm_displacement import set_next_destination

# Import utils and constants
from an_const import ACTIONS_SCORE, DISPLACEMENT

#################################################################
# CONSTANTS
#################################################################

ACTION_TIMEOUT = 5
ARM_TIMING_TAKE = 3
ARM_TIMING_DEPOSE = 3

NEG_ACCURATE_DISP = -30 
POS_ACCURATE_DISP = +15

#################################################################
# TakeCherries - For cherries (perpendicular or against the wall)
#################################################################

class TakeCherries(smach.State) :
    def __init__(self) :
            smach.State.__init__(   self,
                                    outcomes=['fail', 'preempted','done'],
                                    input_keys=['nb_actions_done','cb_arm', 'nb_errors','cherries_loaded'],
                                    output_keys=['nb_actions_done', 'nb_errors','cherries_loaded','cb_arm'])


    def execute(self, userdata):

        #Deploying an arm to take cherries on a rack
        log_info("Ordering take cherries")
        userdata.cb_arm[0] = -1
        cherries_pub.publish(0) # Publish 0 to tell to the BN to deploy the arm in order to recover the cherries
        
        #Wait for the end of action


        time.sleep(3)
        userdata.nb_actions_done[0] += 1
        return 'done'

        begin_time = time.time()
        while (time.time() - begin_time < ARM_TIMING_TAKE) :
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            #React to the feedback
            if userdata.cb_arm[0] == 0 : #Success
                log_info("Take cherries : success")
                userdata.cherries_loaded[0] = 1
                userdata.nb_actions_done[0] += 1
                return 'done'
            elif userdata.cb_arm[0] == 1 : #Error durring the recovery
                log_info("Take cherries : error encountered")
                userdata.nb_errors[0] += 1
                return 'fail'

        if time.time() - begin_time >= ARM_TIMING_TAKE :
            log_info('Timeout reached')
            return action_error("fail")

#####################################################################################
# DepositBucket - Put the cherries in the bucket (DepositCherries was already taken)
#####################################################################################

class DepositBucket(smach.State) :
    def __init__(self) :
        smach.State.__init__(self,
                            outcomes=['preempted','done','fail'],
                            input_keys=['nb_actions_done','cb_arm','cherries_loaded','nb_errors'],
                            output_keys=['nb_actions_done','cherries_loaded','cb_arm','nb_errors'])


    def execute(self, userdata):
        
        #Deploying an arm to depose the cherries
        log_info("Ordering Depose Cherries in the Bucket")
        userdata.cb_arm[0] = -1
        cherries_pub.publish(1) # Publish 1 to tell to the BN to deploy the arm in order to recover the cherries
        
        
        
        time.sleep(3)
        userdata.nb_actions_done[0] += 1
        return 'done'
        
        begin_time = time.time()
        while (time.time() - begin_time < ARM_TIMING_DEPOSE) :
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.1)
            #React to feedback
            if userdata.cb_arm[0] == 0 : #No problem
                log_info("Deposit sample arm : success")
                userdata.nb_actions_done[0] += 1
                return 'done' 
            elif userdata.cb_arm[0] == 1 :
                log_errs("Deposit sample arm : fail")
                userdata.nb_errors[0] += 1
                return 'fail'
            
        if time.time() - begin_time >= ARM_TIMING_TAKE :
            log_info('Timeout reached')
            return action_error("fail")
        
        log_info("No response BN")
        return 'fail'

#TODO : Gestion d'erreur
#########################################################################################################
# TakeCherriesError - Substate managing potential errors induced by taking cherries on a rack
#########################################################################################################

class TakeCherriesPerpendicularError(smach.State) :
    def __init__(self) :
        smach.State.__init__(   self,
                                outcomes=['preempted','done', 'littleDisp'],
                                input_keys=['nb_actions_done','nb_errors','next_pos','cb_arm', 'cb_pos', 'color'],
                                output_keys=['nb_actions_done','nb_errors', 'next_pos','cb_arm'])

    def execute(self, userdata):
        if self.preempt_requested() :
            self.service_preempt()
            return 'preempted'  

        if userdata.cb_arm[0] == 1 :
            if userdata.nb_errors[0] == 1 : 
                x,y,theta = userdata.cb_pos[0]
                set_next_destination(userdata,x,y+POS_ACCURATE_DISP,theta,DISPLACEMENT['accur_av'])
                return 'littleDisp'
            
            elif userdata.nb_errors[0] == 2 :
                x,y,theta = (userdata.cb_pos[0][0], userdata.cb_pos[0][1], userdata.cb_pos[0][2])
                set_next_destination(userdata,x,y+NEG_ACCURATE_DISP,theta,DISPLACEMENT['accur_ar'])
                return 'littleDisp'
            
            elif userdata.nb_errors[0] == 3: # TODO Find n, the number of attempts we can make
                userdata.nb_errors[0] = 0
                userdata.nb_actions_done[0] += 1
                return 'done'
        
        elif userdata.cb_arm[0] == 2 : # We just skip the action
            userdata.nb_errors[0] = 0
            userdata.nb_actions_done[0] += 1
            return 'done'