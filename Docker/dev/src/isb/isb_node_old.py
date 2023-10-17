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
import time
import rospy
import signal
from std_msgs.msg      import Int16, Empty, Int16MultiArray
from geometry_msgs.msg import Quaternion

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[ISB]"

NB_PIXEL = 8
NB_STRAT = 4

# --- LED idx | Uninitialized = stays black 
MATCH_px = [0]
COLOR_px = [1]
STRAT_px = [7,6]  # higher weight pixel on the right... (reverse from usual)


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
# COLORS 
#######################################################################

class pixelColor:
	RD = Quaternion(0, 255, 000, 000)  # red
	GN = Quaternion(0, 000, 255, 000)  # green
	BL = Quaternion(0, 000, 000, 255)  # blue
	WH = Quaternion(0, 255, 255, 255)  # white
	BK = Quaternion(0, 000, 000, 000)  # black
	GY = Quaternion(0, 128, 128, 128)  # gray
	YE = Quaternion(0, 255, 255, 000)  # yellow
	OR = Quaternion(0, 255, 114, 000)  # orange
	PK = Quaternion(0, 223, 112, 173)  # pink
	VT = Quaternion(0,  92, 000, 255)  # violet / purple
	BN = Quaternion(0,  81,  41,  16)  # brown
	CY = Quaternion(0, 000, 255, 255)  # cyan 
	NB = Quaternion(0, 000, 118, 214)  # navy blue

# -- Colors distribution
# MATCH : 	0 = red 	| 1 = green	
# COLOR : 	0 = yellow	| 1 = purple
# STRAT :   0 = black 	| 1 = cyan
MATCH_COLORS = [pixelColor.RD, 	pixelColor.GN]
COLOR_COLORS = [pixelColor.YE, 	pixelColor.VT]
STRAT_COLORS = [pixelColor.BK, 	pixelColor.CY]

STRATS = [0, 1, 2, 3]

#######################################################################
# ISB NODE
#######################################################################

class ISBNode:
	"""
	ISB node.
	"""

	def __init__(self):	
		log_info("Initializing ISB Node ...")

		# --- Variables
		self.match = False
		self.color = -1
		self.strat = -1

		self.isBlinking = [True, True, False, False, False, False, True, True]
		self.off = True

		# --- Publishers & subscribers
		self.pubIsbPixel = rospy.Publisher("/isbSetSinglePixel", Quaternion, queue_size=10, latch=True)
		self.pubReset = rospy.Publisher("/isbReset", Empty, queue_size=10, latch=True)
		self.pubStart = rospy.Publisher("/start", Int16, queue_size=10, latch=True)
		self.pubColor = rospy.Publisher("/color", Int16, queue_size=10, latch=True)
		self.pubStrat = rospy.Publisher("/strat", Int16, queue_size=10, latch=True)

		self.subIsbMatch = rospy.Subscriber("/isbMatchState", Int16, self.matchUpdate)
		self.subIsbColor = rospy.Subscriber("/isbSide", Int16, self.colorUpdate)
		self.subIsbStrat = rospy.Subscriber("/isbStrat", Int16, self.stratUpdate)

		## WHY ??
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)
		self.pubReset.publish(Empty())
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)
		self.setAllPixels(pixelColor.BK)

		self.blinking()

	# --- Pixel handling functions

	def setPixel(self, px_id, color):
		"""Publish pixel color to ISB."""
		self.pubIsbPixel.publish(Quaternion(px_id, color.y, color.z, color.w))

	def setAllPixels(self, color):
		for i in range(NB_PIXEL):
			self.setPixel(i, color)

	def blinking(self):
		# Loop until match start, blink leds red/black
		while not self.match:
			time.sleep(0.5)
			for px in range(NB_PIXEL):
				if not self.isBlinking[px]:
					continue
				if self.off:
					self.setPixel(px, pixelColor.BK)
				else:
					self.setPixel(px, pixelColor.WH)
			self.off = 1 - self.off
		

	# --- Update functions

	def matchUpdate(self, msg):
		"""Update match state, publish on /start the change."""
		if not msg.data in [0, 1]:
			return 

		if msg.data==1:
			self.match = True
			self.pubStart.publish(data=1)
		else:
			self.match = False
			self.pubStart.publish(data=0)

		for px in MATCH_px:
			self.setPixel(px, MATCH_COLORS[msg.data])
			self.isBlinking[px] = not self.match


	def colorUpdate(self, msg):
		"""Update color state, publish on /color the change."""
		if not msg.data in [0, 1]:
			return

		if msg.data==0:
			self.color = 0
			self.pubColor.publish(msg)
		else:
			self.color = 1
			self.pubColor.publish(msg)

		for px in COLOR_px:
			self.setPixel(px, COLOR_COLORS[self.color])
			self.isBlinking[px] = False

		
	def stratUpdate(self, msg):
		"""Update strat, set color special to each one."""
		
		# Relay msg to AN and DN...
		self.pubStrat.publish(msg)

		self.strat = msg.data
		pin0 = int(self.strat % 2)
		pin1 = int(self.strat / 2)

		self.setPixel(STRAT_px[0], STRAT_COLORS[pin0])
		self.setPixel(STRAT_px[1], STRAT_COLORS[pin1])
		self.isBlinking[STRAT_px[0]] = False
		self.isBlinking[STRAT_px[1]] = False
		# for id, px in enumerate(STRAT_px):
		# 	self.setPixel(px, STRAT_COLORS[msg.data[id]])
		# 	self.isBlinking[px] = False
		

#######################################################################
# LAUNCH
#######################################################################

def main():
	rospy.init_node('ISB node')
	signal.signal(signal.SIGINT, sig_handler)
	node = ISBNode()
	rospy.spin()


if __name__ == '__main__':
	main()