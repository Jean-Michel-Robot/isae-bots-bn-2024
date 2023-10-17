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

import os
import rospy
import configparser
from enum import IntEnum, Enum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

NODE_NAME = "[DEC] "
SIMULATION = False if os.environ['HOSTNAME'] in ['pr', 'gr'] else True

#################################################################
# CONFIG 
READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),'../../../gr_config.ini'))

#################################################################
# WINDOW
TERM_SIZE = 62

#################################################################
# ROBOTS PARAMS
class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1

#################################################################

ACTIONS_LIST = [
    'takeCherriesPerpendicular',
    'takeCherriesWall',
    'depositCherries',
    'takeCakes',
    'depositCakes',
    'park',
    'preempted',
    'end',
    'waiting'
    ]

LIST_OF_ACTIONS = {
    'takeCherriesPerpendicular': [0],
    'takeCherriesWall':          [1],
    'depositCherries':           [2],
    'takeCakes':                 [3],
    'depositCakes':              [4],
    'park':                      [5],
    'end':                       [6],
    'waiting':                   [7],
    'depositCherriesNear':       [8],
    'pushCakes':                 [9]
}

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

ACTIONS_SCORE = {
	'init_score':               5, #Bucket posé
    'funnyCounter':            10, #funny action et compte des cerises corrects (5+5)
    'parking':                 15,
    'depositStage':             1,
    'legendary':                4,
    'cherryOnCake':             3,
    'cherryBucket':             1,
    'bonus':                   20
}

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

class Color():
	BLACK = '\033[30m'
	RED = '\033[31m'
	GREEN = '\033[32m'
	YELLOW = '\033[33m'
	BLUE = '\033[34m'
	MAGENTA = '\033[35m'
	CYAN = '\033[36m'
	WHITE = '\033[37m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'
	RESET = '\033[0m'
        
def log_info(log):
    """
    Print standard logs.
    """
    rospy.loginfo(NODE_NAME + log)


def log_warn(log):
    """
    Print warning logs.
    """
    rospy.logwarn(NODE_NAME + log)

def log_errs(log):
    """
    Print errors logs.
    """
    rospy.logerr(NODE_NAME + log)

def log_fatal(log):
    """
    Print errors logs.
    """
    rospy.logfatal(NODE_NAME + log)