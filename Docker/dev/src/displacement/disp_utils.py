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

import configparser
from ast import literal_eval
from pathfinder.obstacle_circ import ObstacleCirc

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

DEBUG_PRINT = True

VERSION = 1
NODE_NAME = "[DSP] "   



READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../../gr_config.ini"))
except:
	print("no file found...")

ROBOT_NAME = READER.get("ROBOT", "robot_name")
INIT_POS = list(literal_eval(READER.get("ROBOT", "init_pos")))
INIT_POS2 = list(literal_eval(READER.get("ROBOT", "init_pos2")))

MAX_ASTAR_TIME = READER.get("PATHFINDER", "max_astar_time")

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

robotW = int(READER.get("ROBOT", "robot_larg"))
robotL = int(READER.get("ROBOT", "robot_long"))
robotDiag = np.linalg.norm([robotW/2, robotL/2])
MARGIN = robotDiag // 2 + 20

CAKES_OBST = {
    0 : ObstacleCirc(225, 575, 60+MARGIN),
    1 : ObstacleCirc(1775, 575, 60+MARGIN),
    2 : ObstacleCirc(225, 2425, 60+MARGIN),
    3 : ObstacleCirc(1775, 2425, 60+MARGIN),
    4 : ObstacleCirc(225, 775, 60+MARGIN),
    5 : ObstacleCirc(1775, 775, 60+MARGIN),
    6 : ObstacleCirc(225, 2225, 60+MARGIN),
    7 : ObstacleCirc(1775, 2225, 60+MARGIN),
    8 : ObstacleCirc(725, 1125, 60+MARGIN),
    9 : ObstacleCirc(1275, 1125, 60+MARGIN),
    10: ObstacleCirc(725, 1875, 60+MARGIN),
    11: ObstacleCirc(1275, 1875, 60+MARGIN)
}

ONE_PI = np.pi
HLF_PI = np.pi/2
QRT_PI = np.pi/4

def to_robot_coord(x_robot, y_robot, cap, pos):
    """Fonction transposant pos dans le repere local du robot."""
    x_loc = np.cos(cap)*(pos[0]-x_robot)+np.sin(cap)*(pos[1]-y_robot)
    y_loc = np.cos(cap)*(pos[1]-y_robot)-np.sin(cap)*(pos[0]-x_robot)
    return (x_loc, y_loc)

def printable_pos(pos):
    """Fonction de mise en forme d'une position pour affichage."""
    p_pos = [int(pos[0]), int(pos[1]), round(pos[2], 2)]
    return p_pos

def patch_frame_br(x, y, theta, color):
    """Easier patch"""
    if COLOR[color] == "HOME":
        return x, y, theta
    return 2000-x, y, theta+ONE_PI

#######################################################################
# LOGS functions
#######################################################################

def log_info(msg):
    """Fonction intermediaire affichant les logs pendant l'execution."""
    rospy.loginfo(NODE_NAME+msg)

def log_errs(msg):
    """Fonction intermediaire affichant les logs d'erreurs."""
    rospy.logerr(NODE_NAME+msg)

def log_warn(msg):
    """Fonction intermediaire affichant les logs de warning."""
    rospy.logwarn(NODE_NAME+msg)


def dprint(msg):
    if DEBUG_PRINT:
        print(" > {}".format(msg))

