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

from an_cste import ROBOT_SIDES, DISP_ORDERS, CB_DISP_MOTION
from an_msgs import next_motion_pub
from an_help import log_info, log_errs, log_warn

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

DISP_TIMEOUT = 30       #[s]
STOP_PATH_TIMEOUT = 4   #[s]
STOP_DEST_TIMEOUT = 3   #[s]

def set_next_destination(userdata, x_d, y_d, t_d, w):
	"""Allows a quick conversion of destination given the side played."""
	if not userdata.color in [e.value for e in ROBOT_SIDES]:
		raise ValueError
	if userdata.color == ROBOT_SIDES.HOME:  
		userdata.next_pos = Quaternion(x_d, y_d, t_d, w)        	
	if userdata.color == ROBOT_SIDES.AWAY:
		userdata.next_pos = Quaternion(x_d, 3000-y_d, -t_d, w)

#################################################################
#                                                               #
#                     SM_DISPLACEMENT STATE                     #
#                                                               #
#################################################################

class SM_Displacement(smach.State):
	"""
	STATE MACHINE : Substate Displacement.

	Handle orders given to the displacement node.
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted','done','redo','fail'],
			input_keys=['nb_actions_done','cb_dsp','cb_pos','next_pos'],
			output_keys=['nb_actions_done','cb_dsp'])
		
	def execute(self, userdata):
		# Init the callback var of dsp result
		userdata.cb_dsp[0] = CB_DISP_MOTION.NONE

		dest = userdata.next_pos
		log_info(f"Displacement Request: toward ({dest.x}, {dest.y}, {dest.z}) with w= {dest.w}")
		next_motion_pub.publish(dest)

		init_time = time.time()
		while (time.time() - init_time < DISP_TIMEOUT):
			time.sleep(0.01)

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			if userdata.cb_dsp[0] == CB_DISP_MOTION.ERROR_ASSERV:
				log_errs("Displacement result: error asserv.")
				# --- try and correct if it's a problem of same position order reject
				curr_x, curr_y, _ = userdata.cb_pos[0]
				if abs(curr_x-dest.x) < 5 and abs(curr_y-dest.y) < 5:
					log_warn("--- error asserv fixed: rotation around same point.")
					set_next_destination(userdata, dest.x, dest.y, dest.z, int(DISP_ORDERS.ROTATION))
					return "redo"	
				return "fail"

			if userdata.cb_dsp[0] == CB_DISP_MOTION.NOPATH_FOUND:
				log_errs("Displacement result: no path found with PF.")
				return 'fail'

			if userdata.cb_dsp[0] == CB_DISP_MOTION.DISP_SUCCESS:
				log_info('Displacement result: success displacement')
				userdata.nb_actions_done[0] += 1
				return 'done'

			if userdata.cb_dsp[0] == CB_DISP_MOTION.PATH_BLOCKED:
				stop_time = time.time()
				while userdata.cb_dsp[0] != CB_DISP_MOTION.DISP_RESTART and time.time()-stop_time < STOP_PATH_TIMEOUT:
					time.sleep(0.01)

					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'

				# Once out of the waiting loop
				if userdata.cb_dsp[0] == CB_DISP_MOTION.DISP_RESTART:	# on est repartis et on attend
					log_info('Displacement restart ...')
					userdata.cb_dsp[0] = CB_DISP_MOTION.NONE
					return 'redo'
				# Else, we are still blocked...
				return 'fail'

			if userdata.cb_dsp[0] == CB_DISP_MOTION.DEST_BLOCKED:
				stop_time = time.time()
				while userdata.cb_dsp[0] != CB_DISP_MOTION.DISP_RESTART and time.time()-stop_time < STOP_DEST_TIMEOUT:
					time.sleep(0.01)

					if self.preempt_requested():
						self.service_preempt()
						return 'preempted'
										
				if userdata.cb_dsp[0] == CB_DISP_MOTION.DISP_RESTART:
					log_info('Displacement restart ...')
					userdata.cb_dsp[0] = CB_DISP_MOTION.NONE
					return 'redo'
			
				log_warn('Displacement result: dest is blocked.')
				return 'fail'

		if time.time() - init_time >= DISP_TIMEOUT :
			log_errs('Timeout reached - [displacement]')
			return 'fail'
			# return actionError('fail')