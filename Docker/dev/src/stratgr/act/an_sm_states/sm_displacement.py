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

from geometry_msgs.msg import Quaternion

from an_const import COLOR, DISPLACEMENT
from an_comm import disp_pub
from an_utils import log_info, log_errs, log_warn, patch_frame_br

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

DISP_TIMEOUT = 30       #[s]
STOP_PATH_TIMEOUT = 3   #[s]
STOP_DEST_TIMEOUT = 3   #[s]

def set_next_destination(userdata, x_d, y_d, t_d, w):
	"""Allows a quick conversion of destination given the side played."""
	x_d, y_d, t_d = patch_frame_br(x_d, y_d, t_d, userdata.color)
	userdata.next_pos = Quaternion(x_d, y_d, t_d, w)

#################################################################
#                                                               #
#                     SM_DISPLACEMENT STATE                     #
#                                                               #
#################################################################

class Displacement(smach.State):
	"""
	STATE MACHINE : Substate Displacement.

	Handle orders given to the displacement node.
	"""

	def __init__(self):
		smach.State.__init__(	self, 	
		       					outcomes=['preempted','done','redo','fail'],
								input_keys=['nb_actions_done','cb_disp','cb_pos','next_pos','color','backward'],
								output_keys=['nb_actions_done','cb_disp','next_pos','cb_pos','backward'])
		
	def execute(self, userdata):
		# Init the callback var of dsp result. CHECK an_const to see details on cb_disp
		userdata.cb_disp[0] = -3 

		dest = userdata.next_pos
		dest2 = userdata.next_pos
		log_info(f"Displacement Request: toward ({dest.x}, {dest.y}, {dest.z}) with w= {dest.w}")
		disp_pub.publish(dest)

		init_time = time.time()
		while (time.time() - init_time < DISP_TIMEOUT):
			time.sleep(0.01)

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			if userdata.cb_disp[0] == -2:
				log_errs("Displacement result: error asserv.")
				# --- try and correct if it's a problem of same position order reject
				curr_x, curr_y, _ = userdata.cb_pos[0]
				""" if abs(curr_x-dest.x) < 5 and abs(curr_y-dest.y) < 5:
					log_warn("--- error asserv fixed: rotation around same point.")
					set_next_destination(userdata, dest.x, dest.y, dest.z, DISPLACEMENT['rotation'])
					return 'redo'	 """
				return 'fail'

			if userdata.cb_disp[0] == -1:
				log_errs("Displacement result: no path found with PF.")
				return 'fail'

			if userdata.cb_disp[0] == 0:
				log_info('Displacement result: success displacement')
				userdata.nb_actions_done[0] += 1
				return 'done'

			if userdata.cb_disp[0] == 1:
				stop_time = time.time()
				while userdata.cb_disp[0] != 2 and time.time()-stop_time < STOP_PATH_TIMEOUT:
					time.sleep(0.01)
					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'

				# Once out of the waiting loop
				if userdata.cb_disp[0] == 2:	# on est repartis et on attend
					log_info('Displacement restart ...')
					userdata.cb_disp[0] = -3
					return 'redo'
				# Else, we are still blocked...
				return 'fail'

			if userdata.cb_disp[0] == 3:
				""" if not userdata.backward :
					userdata.backward = True

					step_x, step_y = 0, 0
					if dest2.x > userdata.cb_pos[0][0] :
						step_x = userdata.cb_pos[0][0] - 50
					else :
						step_x = userdata.cb_pos[0][0] + 50
					if dest2.y > userdata.cb_pos[0][1] :
						step_y = userdata.cb_pos[0][1] - 50
					else :
						step_y = userdata.cb_pos[0][1] + 50
					dest2.x = step_x
					dest2.y = step_y
					dest2.w = DISPLACEMENT['marcheArr']
					log_info("PASSAGE")
					disp_pub.publish(dest2)
					begin_time = time.time()
					while userdata.cb_disp[0] != 0 and time.time()-begin_time < STOP_DEST_TIMEOUT:
						time.sleep(0.1) """
				log_info("RECHERCHE DE CHEMIN")
				userdata.cb_disp[0] = -3
				disp_pub.publish(dest)
				stop_time = time.time()
				
				while userdata.cb_disp[0] != 2 and time.time()-stop_time < STOP_DEST_TIMEOUT:
					time.sleep(0.01)

					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'
					
					if userdata.cb_disp[0] == 3:
						disp_pub.publish(dest)
										
				if userdata.cb_disp[0] == 2:
					log_info('Displacement restart ...')
					userdata.cb_disp[0] = -3
					return 'redo'
			
				log_warn('Displacement result: dest is blocked.')
				return 'fail'

		if time.time() - init_time >= DISP_TIMEOUT :
			log_errs('Timeout reached - [displacement]')
			return 'fail'
			# return actionError('fail')