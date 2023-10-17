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
import numpy as np
import ast
import configparser
from enum import Enum, IntEnum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

_NODENAME_ = "[ACT]"
SIMULATION = False if os.environ['USER'] == 'pi' else True

#-- GAME CONSTANTS --
PARKING_POS = None

#################################################################
#                                                               #
#                            ENUMS                              #
#                                                               #
#################################################################

class ROBOT_SIDES(IntEnum):
    HOME = 0
    AWAY = 1


class DISP_ORDERS(IntEnum):
    STANDARD = 0
    STRAIGHT = 1
    RECAL_AV = 3
    RECAL_AR = 4
    ROTATION = 7
    TOUCH_AV = 12
    TOUCH_AR = 13
    ACCUR_AV = 16
    ACCUR_AR = 15


class ACTS_SCORES(IntEnum):
    INITIAL = 0
    PARKING = 20


class CB_NEXT_ACTION(IntEnum):
    NONE = -3
    STOP = -1
    PARK = -2
    #ACTION_1 = 0
    #ACTION_2 = 0
    #ACTION_3 = 0
    #ACTION_4 = 0
    #ACTION_5 = 0
    #ACTION_6 = 0
    #ACTION_7 = 0
    #...


class CB_DISP_MOTION(IntEnum):
    ERROR_ASSERV = -2
    NOPATH_FOUND = -1
    DISP_SUCCESS = 0
    PATH_BLOCKED = 1
    DISP_RESTART = 2
    DEST_BLOCKED = 3
    NONE         = 9

