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
from an_comm import end_of_action_pub, add_score, pub_delete_obst
from an_sm_states.sm_displacement import Displacement, set_next_destination
from an_sm_states.sm_cakes_substates import *

#################################################################
#                                                               #
#                          SUBSTATES                            #
#                                                               #
#################################################################

class ObsPushCakes(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp','openDoors','closeDoors','redo','openClamp','closeClamp','elevator'],
			                    input_keys=['nb_actions_done','next_pos','color','take_cakes_area','pucks_taken','nb_errors','cb_doors','backward','cb_clamp','cb_elevator','stage_to_go','park','open_clamp','open_doors','elevator_zero','deposit_area'],
			                    output_keys=['nb_actions_done','next_pos','pucks_taken','nb_errors','cb_doors','cb_clamp','cb_elevator','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero','deposit_area'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if userdata.park[0] == 1:
            if userdata.open_doors:
                userdata.open_doors = False
                return 'closeDoors'
            else:
                return 'done'
        
        if userdata.nb_actions_done[0] == 0:
            ## On lance l'action de prendre les palets.
            userdata.open_doors = True
            return 'openDoors'
            
        if userdata.nb_actions_done[0] == 1:
            ## On se déplace jusqu'au site de la pile de gâteaux visée
            x, y, z = DEPOSIT_POS[userdata.deposit_area[0]]
            if userdata.color == 1:
                z = -z
            pub_delete_obst.publish(data=2)
            pub_delete_obst.publish(data=6)
            set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
            return 'disp'

        elif userdata.nb_actions_done[0] == 2:
            log_info("Aaaaah")
            time.sleep(0.5)
            x, y, z = DEPOSIT_POS[userdata.deposit_area[0]]
            y += 150
            set_next_destination(userdata, x, y, z, DISPLACEMENT['marcheArr'])
            return 'disp'

        elif userdata.nb_actions_done[0] == 3:
            userdata.open_doors = False
            return 'closeDoors'

        end_of_action_pub.publish(exit=1, reason='success')
        return 'done' 
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

PushCakes = smach.StateMachine( outcomes=['preempted', 'end'],
                                input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color','cb_doors','cb_clamp','cb_elevator','pucks_taken','nb_errors','take_cakes_area','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero','deposit_area'],
                                output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','pucks_taken','take_cakes_area','nb_errors','cb_doors','cb_clamp','cb_elevator','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero','deposit_area'])
							
with PushCakes:
    smach.StateMachine.add('OBS_PUSH_CAKES', 
                            ObsPushCakes(), 
                            transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT','openDoors':'OPEN_DOORS', 'closeDoors':'CLOSE_DOORS', 'openClamp':'OPEN_CLAMP', 'closeClamp':'CLOSE_CLAMP', 'elevator':'MOVE_ELEVATOR', 'redo':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','redo':'DISPLACEMENT','fail':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('OPEN_DOORS', 
                            OpenDoors(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','fail':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('CLOSE_DOORS', 
                            CloseDoors(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','fail':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('OPEN_CLAMP', 
                            OpenClamp(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','fail':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('CLOSE_CLAMP', 
                            CloseClamp(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','fail':'OBS_PUSH_CAKES'})
    smach.StateMachine.add('MOVE_ELEVATOR', 
                            MoveElevator(), 
                            transitions={'preempted':'preempted','done':'OBS_PUSH_CAKES','fail':'OBS_PUSH_CAKES'})

