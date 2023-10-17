#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: disp_utils.py
@status: clean R3

File containing the utilitary functions and constants used in the disp-
lacement node package.
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################

import os
import rospy
import numpy as np

import ConfigParser
from ast import literal_eval

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

DEBUG_PRINT = True

VERSION = 1
NODE_NAME = "[DSP] "   

SIMULATION = False if os.environ['USER'] == 'pi' else True
ROBOT_NAME = rospy.get_param("robot_name") if SIMULATION else ""

READER = ConfigParser.ConfigParser()
if not SIMULATION: 
    READER.read(os.path.join(os.path.dirname(__file__),"../../start.ini"))
elif ROBOT_NAME == "PR":
    READER.read(os.path.join(os.path.dirname(__file__),"../../pr_start.ini"))
elif ROBOT_NAME == "GR":
    READER.read(os.path.join(os.path.dirname(__file__),"../../gr_start.ini")) 

ROBOT_NAME = READER.get("Robot", "robot_name") # either "PR" or "GR" (easier)

def toRobotCoord(xRobot, yRobot, cap, pos):
    """Fonction transposant pos dans le repere local du robot."""
    xLoc = np.cos(cap)*(pos[0]-xRobot)+np.sin(cap)*(pos[1]-yRobot)
    yLoc = np.cos(cap)*(pos[1]-yRobot)-np.sin(cap)*(pos[0]-xRobot)
    return (xLoc, yLoc)

def printablePos(pos):
    """Fonction de mise en forme d'une position pour affichage."""
    pPos = [int(pos[0]), int(pos[1]), round(pos[2], 2)]
    return pPos

# def patchFrameBR_1(x_r, y_r, cap, x_d, y_d, t_d):
# 	"""Changes the frame so the BR gets us where we want...

# 	Pour cela, il faut :
# 	 - passer au repere local du robot
# 	 - realiser une symetrie par rapport a l'axe xR
# 	 - revenir au repere global HN
# 	 - envoyer a la BR.
     
#     Les parametres sont les suivants:
#      - x_r : x du robot
#      - y_r : y du robot
#      - cap : le cap du robot
#      - x_d ; le x de destination
#      - y_d : le y de destination
#      - t_d : le theta de destination"""

# 	if simulation: 
# 		return x_d, y_d, t_d
# 	# Translation 
# 	x_1 = x_d - x_r
# 	y_1 = y_d - y_r
# 	# Rotation + symetrie (on inverse le y seulement)
# 	x_2 = cos(cap)*x_1 + sin(cap)*y_1
# 	y_2 = sin(cap)*x_1 - cos(cap)*y_1
# 	# Rotation inverse
# 	x_3 = cos(cap)*x_2 - sin(cap)*y_2
# 	y_3 = sin(cap)*x_2 + cos(cap)*y_2
# 	# Translation
# 	x_final = x_3 + x_r
# 	y_final = y_3 + y_r
# 	t_final = pi / 2 - t_d

# 	return x_final, y_final, t_final

def patchFrameBR(x, y, theta):
    """Easier patch"""
    if ROBOT_NAME == "GR":
        return x, y, theta
    return x, 3000-y, -theta

#######################################################################
# LOGS functions
#######################################################################

def LOG_INFO(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(NODE_NAME+msg)

def LOG_ERRS(msg):
    """Fonction intermediaire affichant les logs d'erreurs."""
    rospy.logerr(NODE_NAME+msg)

def LOG_WARN(msg):
    """Fonction intermediaire affichant les logs de warning."""
    rospy.logwarn(NODE_NAME+msg)


def dprint(msg):
    if DEBUG_PRINT:
        print(" > {}".format(msg))

