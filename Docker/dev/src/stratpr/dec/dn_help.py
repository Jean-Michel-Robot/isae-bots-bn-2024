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

import os
import rospy
import configparser
from enum import IntEnum, Enum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

_NODENAME_ = "[DEC]"
SIMULATION = False if os.environ['USER'] == 'pi' else True

#################################################################
# CONFIG 
CONFIG_READER = configparser.ConfigParser()
CONFIG_READER.read(os.path.join(os.path.dirname(__file__),'../../../pr_start.cfg'))

#################################################################
# WINDOW
TERM_SIZE = 62

#################################################################
# ROBOTS PARAMS
class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1

#################################################################
# STRATS PARAMS
class STRAT_INDEX(IntEnum):
    HOMOLOGATION = 0
    TESTS = 1
    MATCH = 2

class STRAT_NAMES(str, Enum):
    HOMOLOGATION = "homologation_strat"
    TESTS = "tests_strat"
    MATCH = "match_strat"

class DN_LIST_ACTION_INDEX(IntEnum):
    PARK = 0
    END = 1
    PREEMPTED = 2
    #...
    #...

class DN_LIST_ACTION_NAMES(str, Enum):
    PARK = 'park'
    END = 'end'
    PREEMPTED = 'preempted'
    #... 

class CB_NEXT_ACTION(IntEnum):
    NONE    = -3
    PARK_IT = -2  # park interrupt
    STOP_IT = -1  # stop interrupt
    #ACTION_1 = 0
    #ACTION_2 = 0
    #ACTION_3 = 0
    #ACTION_4 = 0
    #ACTION_5 = 0
    #ACTION_6 = 0
    #ACTION_7 = 0
    #...



#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

def log_info(msg):
    """
    Print standard logs.
    """
    rospy.loginfo(f"{_NODENAME_} {msg}")


def log_warn(msg):
    """
    Print warning logs.
    """
    rospy.logwarn(f"{_NODENAME_} {msg}")


def log_errs(msg):
    """
    Print errors logs.
    """
    rospy.logerr(f"{_NODENAME_} {msg}")


