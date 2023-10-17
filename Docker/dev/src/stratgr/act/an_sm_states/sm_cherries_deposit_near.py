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
from an_sm_states.sm_cherries_substates import DepositBucket

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsDepositCherriesNear(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp','deposit','redo'],
			                    input_keys=['nb_actions_done','next_pos','color','cherries_loaded','backward','park'],
			                    output_keys=['nb_actions_done','next_pos','cherries_loaded','backward','park'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if userdata.park[0] == 1:
            return 'done'
        
        if userdata.nb_actions_done[0] == 0:
            ## On se déplace jusqu'au site des cerises perpendiculaires au mur (en se mettant dans la bonne orientation pour le bras)
            x, y, z = DEPOSIT_CHERRIES_POS
            y -= ROBOT_LARG/2
            y -= 5
            if userdata.color == 1:
                z = -z
            set_next_destination(userdata, x, y, z, DISPLACEMENT['marcheArr'])
            return 'disp'
        
        """ if userdata.nb_actions_done[0] == 1:
            ## On lance l'action de baisser le bras pour récupérer les cerises et remonter le bras.
            return 'deposit' """
        
        """ if userdata.nb_actions_done[0] == 1:
            ## On se déplace jusqu'au site des cerises perpendiculaires au mur (en se mettant dans la bonne orientation pour le bras)
            x, y, z = DEPOSIT_CHERRIES_POS
            y -= ARM_SHIFT
            if userdata.color == 1:
                z = -z
            set_next_destination(userdata, x, y, z, DISPLACEMENT['noAvoidance'])
            return 'disp' """

        end_of_action_pub.publish(exit=1, reason='success')
        return 'done' 
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

DepositCherriesNear = smach.StateMachine(   outcomes=['preempted', 'end'],
                                        input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color','cb_arm','cherries_loaded','nb_errors','backward','park'],
                                        output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','cherries_loaded','nb_errors','backward','park'])
							
with DepositCherriesNear:
    smach.StateMachine.add('OBS_DEPOSIT_CHERRIES_NEAR', 
                            ObsDepositCherriesNear(), 
                            transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT','deposit':'DEPOSIT_CHERRIES', 'redo':'OBS_DEPOSIT_CHERRIES_NEAR'})
    smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
                            transitions={'preempted':'preempted','done':'OBS_DEPOSIT_CHERRIES_NEAR','redo':'DISPLACEMENT','fail':'OBS_DEPOSIT_CHERRIES_NEAR'})
    smach.StateMachine.add('DEPOSIT_CHERRIES', 
                            DepositBucket(), 
                            transitions={'preempted':'preempted','done':'OBS_DEPOSIT_CHERRIES_NEAR','fail':'OBS_DEPOSIT_CHERRIES_NEAR'})

