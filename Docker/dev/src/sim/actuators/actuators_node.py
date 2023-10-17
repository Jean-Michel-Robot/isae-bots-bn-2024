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

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import math
from time import sleep
import rospy
import random
from std_msgs.msg      import Int16
from geometry_msgs.msg import Pose2D


#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

NODE_NAME = "[ACN] "

def log_info(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(NODE_NAME+msg)

def log_warn(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.log_warn(NODE_NAME+msg)

## Constants PR

READ_TIME = 0.5
READ_FAIL_PROB = 0  # entre 0 et 1

## Constants GR
CHERRIES_TAKING = 0
DOORS_TIME = 0
CLAMP_TIME = 0
ELEVATOR_TIME = 0

#############################

#######################################################################
## DATA to put in .ini file in section [Simu]
EXCAVATION_SQUARE_POS_X = 1850
EXCAVATION_SQUARE_POS_Y = [667.5, 852.5, 1037.5, 1222.5, 1407.5, 1592.5, 1777.5, 1962.5, 2147.5, 2332.5]

Y_ARM_OFFSET = 40

x_threshold = 50
y_threshold = 30
c_threshold = 1.
#######################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}


## ACTUATOR Node ######################################################
class ActuatorNode():

	"""Node used during simulations to send callbacks instead of the actuators."""

	def __init__(self):		

		#### Communication - pubs & subs ####
		# Sub a /color pour s'initialiser au set de la couleur
		self.sub_color = rospy.Subscriber("/game/color", Int16, self.update_color)
		self.sub_pos = rospy.Subscriber("/current_position", Pose2D, self.update_position)
		
		# Simule la reponse du BN sur le bras à cerises
		self.cherries_sub = rospy.Subscriber('/strat/cherries', Int16, self.cherries_response)
		self.cherries_pub = rospy.Publisher("/strat/cherries_feedback", Int16, queue_size=10, latch=True) 

		# Simule la reponse du BN sur les portes
		self.doors_sub = rospy.Subscriber('/strat/doors', Int16, self.doors_response)
		self.doors_pub = rospy.Publisher("/strat/doors_feedback", Int16, queue_size=10, latch=True)  

		# Simule la reponse du BN sur la pince
		self.clamp_sub = rospy.Subscriber("/strat/clamp", Int16, self.clamp_response)
		self.clamp_pub = rospy.Publisher("/strat/clamp_feedback", Int16, queue_size=10)

		# Simule la réponse du BN sur l'ascenceur
		self.elevator_sub = rospy.Subscriber("/strat/elevator", Int16, self.elevator_response)
		self.elevator_pub = rospy.Publisher("/strat/elevator_feedback", Int16, queue_size=10)

		# Comm avec l'interface de simulation
		self.square_layout_pub = rospy.Publisher("/simu/squareLayout", Int16, queue_size=10, latch=True)
		self.square_info_pub = rospy.Publisher("/simu/squareInfo", Int16, queue_size=10, latch=True)
        
        

		#### Variables ####

		self.color = 0  							# par défaut
		self.curr_pos = Pose2D(x=0, y=0, theta=0)	# par defaut

		self.info_square = [-1] * 7  				# init undefined
		self.curr_square = 0


		# TODO : en constantes
		self.excavationSquarePos_x = 1870  # TODO : a paramétrer
		self.excavationSquarePos_y = [667.5, 852.5, 1037.5, 1222.5, 1407.5, 1592.5, 1777.5, 1962.5, 2147.5, 2332.5]


		# Publication continue s'il y a besoin
		# while(True):
		# 	time.sleep(0.01)


	###################################################################
	# CALLBACKS
	###################################################################

	def update_color(self, msg):
		"""Callback de couleur."""
		self.color = msg.data

		# Generation aleatoire des carres de fouille
		log_info("Random generation of excavation squares.")

	def update_position(self, msg):
		"""Callback de position."""
		self.curr_pos = msg

	def cherries_response(self, msg):
		sleep(CHERRIES_TAKING)
		if msg.data == 0:
			self.cherries_pub.publish(data=0)
			log_info("Réponse simulée : cerises récupérées")
		else:
			self.cherries_pub.publish(data=0)
			log_info("Réponse simulée : cerises déposées")

	def doors_response(self, msg):
		sleep(DOORS_TIME)
		if msg.data == 1:
			self.doors_pub.publish(data=0)
			log_info("Réponse simulée : Portes ouvertes")
		else:
			self.doors_pub.publish(data=0)
			log_info("Réponse simulée : Portes fermées")

	def clamp_response(self, msg):
		sleep(CLAMP_TIME)
		if msg.data == 1:
			self.clamp_pub.publish(data=0)
			log_info("Réponse simulée : Pince ouverte")
		else:
			self.clamp_pub.publish(data=0)
			log_info("Réponse simulée : Pince fermée")

	def elevator_response(self, msg):
		sleep(CHERRIES_TAKING)
		if msg.data in range(0,9):
			self.elevator_pub.publish(data=1)
			log_info("Réponse simulée : ascenseur déplacé")
		else:
			self.elevator_pub.publish(data=0)
			log_info("Problème sur l'étage demandé")


#################################################################
#																#
# 							Main 								#
#																#
#################################################################
def main():
	rospy.init_node("pr_bn_response")
	node = ActuatorNode()
	log_info("Initializing Actuator Node.")
	log_info("Simulation of BN responses.")
	rospy.spin()


if __name__ == '__main__':
	main()
