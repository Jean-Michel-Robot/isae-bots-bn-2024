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

class ObsTakeCakes(smach.State):
    """
    SM PARK : Observer state
    """
    def __init__(self):
        smach.State.__init__(   self,  
                                outcomes=['preempted','done','disp','openDoors','closeDoors','redo','openClamp','closeClamp','elevator'],
			                    input_keys=['nb_actions_done','next_pos','color','take_cakes_area','pucks_taken','nb_errors','cb_doors','backward','cb_clamp','cb_elevator','stage_to_go','park','open_clamp','open_doors','elevator_zero'],
			                    output_keys=['nb_actions_done','next_pos','pucks_taken','nb_errors','cb_doors','cb_clamp','cb_elevator','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        """ if userdata.park[0] == 1:
            if userdata.open_clamp:
                if userdata.elevator_zero:  
                    userdata.open_clamp = False
                    return 'closeClamp'
                else :
                    userdata.elevator_zero = True
                    userdata.stage_to_go[0] = 0
                    return 'elevator'
            if not userdata.open_clamp and userdata.elevator_zero :
                userdata.stage_to_go[0] = 1
                userdata.elevator_zero = False
                return 'elevator' 
            if userdata.open_doors:
                userdata.open_doors = False
                return 'closeDoors'
            return 'done' """
        
        if userdata.park[0] == 1:
            if userdata.open_doors:
                userdata.open_doors = False
                return 'closeDoors'
            else:
                return 'done'
        
            
        if userdata.nb_actions_done[0] == 0:
            ## On se déplace jusqu'au site de la pile de gâteaux visée
            x, y, z = CAKES_POS[userdata.take_cakes_area[0]]
            change = False
            if userdata.take_cakes_area[0] == 12:
                change = True 
                userdata.take_cakes_area[0] = 10
            pub_delete_obst.publish(data=userdata.take_cakes_area[0])
            if change :
                userdata.take_cakes_area[0] = 12
            """ # Modif pour le dernier match on le prend par le bas
            if x < MAX_X/2 :
                x += DOORS_SHIFT
            else :
                x -= DOORS_SHIFT """
            y -= DOORS_SHIFT
            if userdata.color == 1:
                z = -z
            set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
            return 'disp'
        
        elif userdata.nb_actions_done[0] == 1:
            ## On lance l'action de prendre les palets.
            userdata.open_doors = True
            return 'openDoors'

        elif userdata.nb_actions_done[0] == 2:
            x, y, z = CAKES_POS[userdata.take_cakes_area[0]]
            if userdata.color == 1:
                z = -z
            set_next_destination(userdata, x, y, z, DISPLACEMENT['standard'])
            return 'disp'

        elif userdata.nb_actions_done[0] == 3:
            userdata.open_doors = False
            return 'closeDoors'
        
        """ elif userdata.nb_actions_done[0] == 1:
            userdata.nb_actions_done[0] += 1
            if userdata.elevator_zero:
                userdata.elevator_zero = False
                userdata.stage_to_go[0] = 8
                return 'elevator' """
                

        
        
        """ if userdata.pucks_taken[0] > 0 :    
            if userdata.nb_actions_done[0] == 3:
                userdata.stage_to_go[0] = 3
                userdata.elevator_zero = False
                return 'elevator'
            
            elif userdata.nb_actions_done[0] == 4:
                userdata.open_clamp = True
                return 'openClamp'
            
            elif userdata.nb_actions_done[0] == 5:
                userdata.stage_to_go[0] = 0
                userdata.elevator_zero = True
                return 'elevator'
            
            elif userdata.nb_actions_done[0] == 6:
                userdata.open_clamp = False
                return 'closeClamp'
            
            elif userdata.nb_actions_done[0] == 7:
                userdata.pucks_taken[0] += 3
                userdata.stage_to_go[0] = 9-userdata.pucks_taken[0]
                userdata.elevator_zero = (userdata.stage_to_go[0] == 0)
                return 'elevator'
            
            elif userdata.nb_actions_done[0] == 8:
                userdata.open_doors = False
                return 'closeDoors'
            
        else :

            if userdata.nb_actions_done[0] == 3:
                userdata.open_clamp = True
                return 'openClamp'
            
            elif userdata.nb_actions_done[0] == 4:
                userdata.stage_to_go[0] = 0
                userdata.elevator_zero = True
                return 'elevator'
            
            elif userdata.nb_actions_done[0] == 5:
                userdata.open_clamp = False
                return 'closeClamp'
            
            elif userdata.nb_actions_done[0] == 6:
                userdata.pucks_taken[0] += 3
                userdata.nb_actions_done[0] += 1
                userdata.stage_to_go[0] = 9-userdata.pucks_taken[0]
                userdata.elevator_zero = (userdata.stage_to_go[0] == 0)
                return 'elevator' """

        end_of_action_pub.publish(exit=1, reason='success')
        return 'done' 
                  

       


#################################################################
#                                                               #
#                        SM STATE : PARK                        #
#                                                               #
#################################################################

TakeCakes = smach.StateMachine( outcomes=['preempted', 'end'],
                                input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos', 'color','cb_doors','cb_clamp','cb_elevator','pucks_taken','nb_errors','take_cakes_area','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero'],
                                output_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','pucks_taken','take_cakes_area','nb_errors','cb_doors','cb_clamp','cb_elevator','stage_to_go','backward','park','open_clamp','open_doors','elevator_zero'])
							
with TakeCakes:
    smach.StateMachine.add('OBS_TAKE_CAKES', 
                            ObsTakeCakes(), 
                            transitions={'preempted':'preempted','done':'end','disp':'DISPLACEMENT','openDoors':'OPEN_DOORS', 'closeDoors':'CLOSE_DOORS', 'openClamp':'OPEN_CLAMP', 'closeClamp':'CLOSE_CLAMP', 'elevator':'MOVE_ELEVATOR', 'redo':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('DISPLACEMENT', 
                            Displacement(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','redo':'DISPLACEMENT','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('OPEN_DOORS', 
                            OpenDoors(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('CLOSE_DOORS', 
                            CloseDoors(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('OPEN_CLAMP', 
                            OpenClamp(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('CLOSE_CLAMP', 
                            CloseClamp(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})
    smach.StateMachine.add('MOVE_ELEVATOR', 
                            MoveElevator(), 
                            transitions={'preempted':'preempted','done':'OBS_TAKE_CAKES','fail':'OBS_TAKE_CAKES'})

