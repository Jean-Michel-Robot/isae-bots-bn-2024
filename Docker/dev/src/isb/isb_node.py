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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import sys
from time import perf_counter, sleep
import rospy
import signal
from std_msgs.msg      import Int16, Empty, Int16MultiArray
from geometry_msgs.msg import Quaternion

from isb_hal import initPin, readPin, writePin

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[ISB]"



def log_info(msg):
	"""
	Print logs standard.
	"""
	rospy.loginfo(f"{_NODENAME_} {msg}")


def log_warn(msg):
	"""
	Print logs warning.
	"""
	rospy.logwarn(f"{_NODENAME_} {msg}")


def log_errs(msg):
	"""
	Print logs errors.
	"""
	rospy.logerr(f"{_NODENAME_} {msg}")


def sig_handler(s_rcv, frame):
    """
    Force node to quit on SIGINT.
    """
    log_warn("Node forced to terminate ...")
    rospy.signal_shutdown(signal.SIGTERM)
    sys.exit()



#######################################################################
# ISB NODE
#######################################################################

REFRESH_PERIOD = 0.05 # s


NB_LEDS = 10
NB_BUTTONS = 5


LED_PINS = [15,14,13,12,11,10,9,8,7,6]
BUTTONS_PINS = [0,1,2,3,4]
TRIGGER_PIN = 5

BUTTON_LEDS_IDS = [9,8,7,6,5]
TRIGGER_LED_ID = 0
BR_IDLE_LED_ID = 1

COLOR_BUTTON_ID = 1
BR_IDLE_BUTTON_ID = 2
RESET_STEPPER_BUTTON_ID = 3

class ISBNode:
	"""
	ISB node.
	"""

	def __init__(self):	
		log_info("Initializing ISB Node ...")


		# --- Variables
		self.currentTime = 0.0

		self.match = False
		self.color = 0

		self.buttonStates = [-1]*NB_BUTTONS
		self.isButtonTriggered = [False]*NB_BUTTONS
		self.triggerState = -1

		self.brIdle = -1

		self.isBlinking = [True, True, False, False, False, False, True, True]

		# --- Publishers & subscribers
		self.pubStart = rospy.Publisher("/game/start", Int16, queue_size=10, latch=True)
		self.pubColor = rospy.Publisher("/game/color", Int16, queue_size=10, latch=True)

		self.pubBRIdle = rospy.Publisher("/br/idle", Int16, queue_size=10, latch=True)
		self.pubResetStepper = rospy.Publisher("/strat/elevator", Int16, queue_size=10, latch=True)

		self.subIsbMatch = rospy.Subscriber("/okPosition", Int16, self.callBackBR)

		self.initPins()
		log_info("Pin initiallized")

		self.setAllLeds(0)		

		log_info("ISB node is ready")



	# --- Pixel handling functions

	def initPins(self):

		for id_pin in LED_PINS:
			initPin(id_pin, "output")

		for id_pin in BUTTONS_PINS:
			initPin(id_pin, "input")

		initPin(TRIGGER_PIN, "input")



	def writeLed(self, led_id, state):
		'''state of 0 or 1'''
		
		writePin(LED_PINS[led_id], state)

	
	def setAllLeds(self, state):
		for k in range(NB_LEDS):
			self.writeLed(k, state)


	def readButtons(self):

		for k in range(NB_BUTTONS):
			res = readPin(BUTTONS_PINS[k])

			# update button triggered list
			self.isButtonTriggered[k] = self.buttonStates[k] != res
			self.buttonStates[k] = res

			# apply state to corresponding led
			self.writeLed(BUTTON_LEDS_IDS[k], self.buttonStates[k])


	def readTrigger(self):

		res = readPin(TRIGGER_PIN)

		# only change trigger state if previous state is not -1
		if res == 1:
			self.triggerState = 1
			self.writeLed(TRIGGER_LED_ID, self.triggerState)  # trigger is ready to be pulled

		if self.triggerState == 1 and res == 0:
			self.triggerState = 0  # triggers match starts


	# --- Callback functions

	def callBackBR(self, msg):

		if msg.data == 5:  # OK_READY
			self.writeLed(BR_IDLE_LED_ID, 1)

		elif msg.data == 6:  # OK_IDLE
			self.writeLed(BR_IDLE_LED_ID, 0)



	def run(self):

		while True:

			if perf_counter() - self.currentTime > REFRESH_PERIOD:

				self.readButtons()
				self.readTrigger()

				# Send match message if the trigger is released
				if  not self.match and self.triggerState == 0:
					log_info("Match started")
					self.match = True
					self.pubStart.publish(data=1)
					self.pubStart.publish(data=1)  # another one to be sure


				# Send color message
				if self.isButtonTriggered[COLOR_BUTTON_ID]:

					if self.color == 0 and self.buttonStates[COLOR_BUTTON_ID] == 1:
						log_info("Set color to AWAY")
						self.color = 1
						self.pubColor.publish(data=1)


					elif self.color == 1 and self.buttonStates[COLOR_BUTTON_ID] == 0:
						log_info("Set color to HOME")
						self.color = 0
						self.pubColor.publish(data=0)


				# Send BR idle message
				if self.isButtonTriggered[BR_IDLE_BUTTON_ID]:

					if self.brIdle == 1 and self.buttonStates[BR_IDLE_BUTTON_ID] == 0:
						log_info("Set BR to Idle")
						self.brIdle = 0
						self.pubBRIdle.publish(data=0)

					elif self.brIdle == 0 and self.buttonStates[BR_IDLE_BUTTON_ID] == 1:
						log_info("Set BR to Ready")
						self.brIdle = 1
						self.pubBRIdle.publish(data=1)
					else:
						self.brIdle = self.buttonStates[BR_IDLE_BUTTON_ID]

				# Reset stepper message
				if self.isButtonTriggered[RESET_STEPPER_BUTTON_ID]:
					log_info("Reset stepper position")
					self.pubResetStepper.publish(data=-1)

				self.currentTime = perf_counter()

			sleep(0.001)
		

#######################################################################
# LAUNCH
#######################################################################

def main():
	rospy.init_node('isb_node')
	signal.signal(signal.SIGINT, sig_handler)

	node = ISBNode()

	# main loop
	node.run()

	rospy.spin()


if __name__ == '__main__':
	main()