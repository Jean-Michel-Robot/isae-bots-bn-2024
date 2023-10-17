#!/usr/bin/env python3
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

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import sys

import os
PATH = os.path.dirname(os.path.abspath(__file__))

import signal
import rospy
import time

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QTimer

from gui_msgs import init_msgs
from gui_utils import log_info, log_warn, log_errs

from mvc.Model import Model
from mvc.View import View
from mvc.Controller import Controller



REFRESH_PERIOD = 10  # Hz

REFRESH_FREQUENCY = 1/REFRESH_PERIOD


def sig_handler(s_rcv, frame):
	"""
	Force node to quit on SIGINT
	"""
	print("Quit on SIGINT")

	rospy.signal_shutdown(signal.SIGTERM)
	sys.exit()



#################################################################
#                                                               #
#                           GUI node                            #
#                                                               #
#################################################################

class GuiNode:
	"""
	GUI node: ROS node to display what happens with the robot
	"""

	# Imported methods
	from gui_msgs import init_msgs, update_start, update_color, update_position

	def __init__(self):

		log_info("Initializing GUI node ...")
		print("Init")

		init_msgs(self)


		# Creation of the gui application
		self.app = QApplication(sys.argv)
		self.app.setWindowIcon(QIcon(os.path.join(PATH, 'image_files/icon.png')))
		self.app.setApplicationName("Robot Simulator")
		




		# Creation of mvc instances
		self.model = Model()  # the model contains all the match data
		self.view = View(self.model)  # the view draws the model periodically
		self.controller = Controller(self.model, self.view)  # the controller updates the model aperiodically (ROS msgs)

		# self.run()
		# self.timer = QTimer()
		# self.timer.timeout.connect(self.run)
		# self.timer.start(int(1000/REFRESH_FREQUENCY))


	
	def run(self):

		# current_time = time.time()

		# # refresh loop (able to pause it and resume it)
		# while True:
		# 	if time.time() - current_time > REFRESH_FREQUENCY:
		# 		current_time = time.time()

		# 		self.controller.refreshMatchView()
		# 	print("ting")

		# 	time.sleep(0.01)

		print("lol")






		


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
	#############################################################
	# INITIALIZATION
	#############################################################

	signal.signal(signal.SIGINT, sig_handler)
	rospy.init_node('GUI')   

	time.sleep(1)  # TODO : delay for rostopic echo command to setup before we log anything (OK if we can afford this 1 second delay)

	node = GuiNode()
	
	screenDims = node.app.primaryScreen().size()
	node.view.setupScreenDims(screenDims)
	node.view.setOrientation(True)

	node.view.show()

	log_info("Executing app")
	try:
		sys.exit(node.app.exec_())
	except SystemExit:
		print('Interface exited')    

	#rospy.spin()  # TODO : need it ? exec_() is blocking


if __name__ == '__main__':
	main()
