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
from ast import literal_eval
import configparser
from enum import Enum, IntEnum

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

NODE_NAME = "[ACT] "
SIMULATION = False if os.environ['HOSTNAME'] in ['pr', 'gr'] else True

#-- GAME CONSTANTS --

READER = configparser.ConfigParser()
try :
	READER.read(os.path.join(os.path.dirname(__file__),"../../../gr_config.ini"))
except:
	print("no file found...")

#################################################################
#                                                               #
#                          ROBOT                                #
#                                                               #
#################################################################

## Park Position
PARKING_POS = list(literal_eval(READER.get('ROBOT', 'park_pos')))

## Origin Position 
ORIGIN = list(literal_eval(READER.get('ROBOT','init_pos')))

MAX_X = 2000
MAX_Y = 3000

ROBOT_LARG = int(READER.get('ROBOT', 'robot_larg'))
ROBOT_LONG = int(READER.get('ROBOT', 'robot_long'))
ROBOT_DIAG = np.sqrt(ROBOT_LARG**2 + ROBOT_LONG**2) 

DOORS_SHIFT = ROBOT_DIAG//2 + 30
ARM_SHIFT = ROBOT_DIAG//2 + 30

ONE_PI = np.pi
TWO_PI = np.pi * 2
HLF_PI = np.pi / 2
QRT_PI = np.pi / 4
SQRT_2 = np.sqrt(2)

#################################################################
#                                                               #
#                       SM CONSTANTS                            #
#                                                               #
#################################################################

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

DISPLACEMENT = {
      'standard'         : 0, 
      'noAvoidance'      : 1,
      'stop'             : 2,
      'accurate'         : 3,
      'recalage'         : 4,
      'rotation'         : 5,
      'marcheArr'        : 8
}

CB_DISP = {
	-3: 'None',
	-2: 'Error Asserv',
    -1: 'Path not found',
     0: 'Disp Success',
     1: 'Path Blocked',
     2: 'Restart',
     3: 'Destination blocked'
}

#TODO : Compléter ces listes au fur et à mesure de l'avancement de l'AN

## I/O keys for states of the sm
ALL_KEY_LIST = [
    'start',
    'color',
    'score',
    'nb_actions_done',
    'cb_disp',
    'cb_arm',
    'cb_elevator',
    'cb_clamp',
    'cb_doors',
    'cb_pos',
    'arm_order',
    'depositArea',
    'next_action',
    'next_pos',
    'deposit_area',
    'take_cakes_area',
    'take_cherries_area',
    'pucks_taken',
    'cherries_loaded',
    'error_reaction',
    'nb_errors',
    'stage_to_go',
    'stage_to_deposit',
    'backward',
    'park',
    'open_clamp',
    'open_doors',
    'elevator_zero'
    ]

ACTIONS_LIST = [
    'takeCherriesPerpendicular',
    'takeCherriesWall',
    'depositCherries',
    'takeCakes',
    'depositCakes',
    'park',
    'end',
    'waiting',
    'depositCherriesNear',
    'pushCakes',
    'preempted'
    ]

ACTIONS_STATES = {
    'takeCherriesPerpendicular':'TAKE_CHERRIES_PERPENDICULAR',
    'takeCherriesWall':'TAKE_CHERRIES_WALL',
    'depositCherries':'DEPOSIT_CHERRIES',
    'depositCherriesNear':'DEPOSIT_CHERRIES_NEAR',
    'takeCakes':'TAKE_CAKES',
    'depositCakes':'DEPOSIT_CAKES',
    'park':'PARK',
    'preempted':'END',
    'end':'END',
    'waiting':'WAITING',
    'pushCakes':'PUSH_CAKES'
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

CHERRIES_POS = {
	0: [985, 2850, ONE_PI],
	1: [985, 150, ONE_PI],
	2: [15, 1500, 0],
	3: [1985, 1500, ONE_PI]
}

DEPOSIT_CHERRIES_POS = [225, 3000, -HLF_PI]

CAKES_POS = {
	0: [225, 575, ONE_PI],  # Rose 
	1: [1775, 575, 0],
	2: [225, 2425, ONE_PI],
	3: [1775, 2425, 0],
	4: [225, 775, ONE_PI],  # Jaune
	5: [1775, 775, 0], 
	6: [225, 2225, ONE_PI],
	7: [1775, 2225, 0], 
	8: [725, 1125, ONE_PI], # Marron
	9: [1275, 1125, 0],
	10:[725, 1875, ONE_PI],
	11:[1275, 1875, 0],
	12:[725, 1875, HLF_PI] # Le même que le 10 mais pour le dernier match on le prend par le bas
}

DEPOSIT_POS = {
	0: [725, 225, -HLF_PI],
	1: [1775, 225, -HLF_PI],
	2: [225, 1125, -HLF_PI], # Normalement ONE_PI mais dernier match donc osef
	3: [1775, 1875, 0],
	4: [225, 2750, HLF_PI],
	5: [350, 1200, ONE_PI]
}
