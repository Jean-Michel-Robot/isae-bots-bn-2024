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
import sys
import time
import smach
from an_const import *
from an_comm import end_of_action_pub, add_score
from an_sm_states.sm_displacement import Displacement, set_next_destination

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsWaiting(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','redo'],
			                    input_keys=['nb_actions_done','next_pos','color'],
			                    output_keys=['nb_actions_done','next_pos'])

    def execute(self, userdata):
        begin_time = time.time()

        while time.time() - begin_time < 100:
            time.sleep(0.01)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'       
        ## Return --> Repartitor
        # endOfAction_pub.publish(exit=1, reason='success')
        return 'redo'
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

Waiting = smach.StateMachine(   outcomes=['preempted', 'end'],
                                input_keys=['nb_actions_done','next_pos', 'color'],
                                output_keys=['nb_actions_done','next_pos','color'])
							
with Waiting:
    smach.StateMachine.add('OBS_WAITING', 
                            ObsWaiting(), 
                            transitions={'preempted':'preempted','done':'end','redo':'OBS_WAITING'})
   

