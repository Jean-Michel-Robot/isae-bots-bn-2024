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
@file: obstacles_creator.py
@status: in progress

TODO: add samples as obstacles
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import numpy as np

from ast import literal_eval
from pathfinder.obstacle_rect import ObstacleRect
from pathfinder.obstacle_circ import ObstacleCirc
from pathfinder.obstacle_tria import ObstacleTria

from disp_utils import *

HOME = 0
AWAY = 1

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def make_obstacle_list(color):
    """Fonction retournant une liste d'obstacles statiques."""

    ## STATIC OBSTACLES ###############################################

    # On rentre les obstacles avec cotes precises et on y rajoute la 
    # largeur du robot qu'on simule
    robotW = int(READER.get("ROBOT", "robot_larg"))
    robotL = int(READER.get("ROBOT", "robot_long"))
    robotDiag = np.linalg.norm([robotW/2, robotL/2])
    margin = robotDiag // 2 + 20

    # Walls 
    wallNorth = ObstacleRect(margin, margin, margin, 3000-margin)
    wallSouth = ObstacleRect(2000-margin, 2000-margin, margin, 3000-margin)
    wallEast = ObstacleRect(margin, 2000-margin, margin, margin)
    wallWest = ObstacleRect(margin, 2000-margin, 3000-margin, 3000-margin)
    """ wallNorth = ObstacleRect(0, 0, 0, 3000)
    wallSouth = ObstacleRect(2000, 2000, 0, 3000)
    wallEast = ObstacleRect(0, 2000, 0, 0)
    wallWest = ObstacleRect(0, 2000, 3000, 3000) """

    # Bases
    baseHome = ObstacleRect(400-margin, 1000+margin, 0, 400+margin)
    baseAway = ObstacleRect(400-margin, 1000+margin, 2600-margin, 3000)

    # Cherries
    cherriesPerpendicular1 = ObstacleRect(985-margin, 1015+margin, 0, 300+margin)
    cherriesPerpendicular2 = ObstacleRect(985-margin, 1015+margin, 2700-margin, 3000)
    cherriesWall1 = ObstacleRect(0, 30+margin, 1350-margin, 1650+margin)
    cherriesWall2 = ObstacleRect(1970-margin, 2000, 1350-margin, 1650+margin)

    # Cakes
    cakes1 = ObstacleCirc(225, 575, 60+margin)
    cakes2 = ObstacleCirc(1775, 575, 60+margin)
    cakes3 = ObstacleCirc(225, 2425, 60+margin)
    cakes4 = ObstacleCirc(1775, 2425, 60+margin)
    cakes5 = ObstacleCirc(225, 775, 60+margin)
    cakes6 = ObstacleCirc(1775, 775, 60+margin)
    cakes7 = ObstacleCirc(225, 2225, 60+margin)
    cakes8 = ObstacleCirc(1775, 2225, 60+margin)
    cakes9 = ObstacleCirc(725, 1125, 60+margin)
    cakes10= ObstacleCirc(1275, 1125, 60+margin)
    cakes11= ObstacleCirc(725, 1875, 60+margin)
    cakes12= ObstacleCirc(1275, 1875, 60+margin)

    # Test
    test = ObstacleRect(1250, 1950, 1125, 1200)

    # Samples
    # s_radius = 150 / 2
    # s_margin = 10

    # samplesCoor = [(555, 900), (675, 830), (795, 900)]
    samplesList = []
    # for x,y in samplesCoor:
    #     samplesList.append( ObstacleCirc(x,y,s_radius+s_margin+margin))

    obstacleList = samplesList

    if color == HOME:
        obstacleList.extend([wallNorth, wallEast, wallWest, wallSouth])
        obstacleList.extend([cherriesPerpendicular1, cherriesPerpendicular2, cherriesWall1, cherriesWall2])
        obstacleList.extend([cakes1, cakes2, cakes3, cakes4, cakes5, cakes6, cakes7, cakes8, cakes9, cakes10, cakes11, cakes12])
        #obstacleList.extend([test])

        #obstacleList.append(baseHome)
    else:
        obstacleList.extend([wallNorth, wallEast, wallWest, wallSouth])
        obstacleList.extend([cherriesPerpendicular1, cherriesPerpendicular2, cherriesWall1, cherriesWall2])
        obstacleList.extend([cakes1, cakes2, cakes3, cakes4, cakes5, cakes6, cakes7, cakes8, cakes9, cakes10, cakes11, cakes12])
        #obstacleList.extend([test])

        #obstacleList.append(baseAway)

    log_info("Number of static obstacles : {}.".format(len(obstacleList)))
    return obstacleList
