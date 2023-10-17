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
from an_cste import DISP_ORDERS, ACTS_SCORES, PARKING_POS
from an_msgs import done_action_pub, send_added_score
from an_sm_states.sm_displacement import SM_Displacement, set_next_destination

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class SM_ObsPark(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted','done','move'],
			input_keys=['nb_actions_done'],
			output_keys=['nb_actions_done'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ## Move to parking position
        x, y, z = PARKING_POS
        if userdata.nb_actions_done[0] == 0:
            set_next_destination(userdata, x, y, z, DISP_ORDERS.STANDARD)
            return 'disp'

        send_added_score(int(ACTS_SCORES.PARKING))
        done_action_pub.publish(exit=1, reason='success')
        return 'done'


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

SM_Park = smach.StateMachine(outcomes=['preempted', 'end'],
			input_keys=['nb_actions_done','cb_dsp','cb_pos','next_pos'],
			output_keys=['nb_actions_done','cb_dsp'])
							
with SM_Park:
	smach.StateMachine.add('OBS_PARK', SM_ObsPark(), 
		transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT'})
	smach.StateMachine.add('DISPLACEMENT', SM_Displacement(), 
		transitions={'preempted':'preempted','done':'OBS_PARK','redo':'DISPLACEMENT','fail':'OBS_PARK'})

