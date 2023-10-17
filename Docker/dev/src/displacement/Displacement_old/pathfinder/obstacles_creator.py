#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
from obstacle_rect import ObstacleRect
from obstacle_circ import ObstacleCirc
from obstacle_tria import ObstacleTria

from disp_utils import READER
from disp_utils import LOG_INFO

HOME = 0
AWAY = 1

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def makeObstacleList(color):
    """Fonction retournant une liste d'obstacles statiques."""

    ## STATIC OBSTACLES ###############################################

    # On rentre les obstacles avec cotes precises et on y rajoute la 
    # largeur du robot qu'on simule
    robotW = int(READER.get("Robot", "robot_larg"))
    robotL = int(READER.get("Robot", "robot_long"))
    robotDiag = np.linalg.norm([robotW/2, robotL/2])
    margin = robotDiag // 2 + 20

    # Galeries
    galerieHome = ObstacleRect(0, 100+margin, 450, 1170)
    galerieAway = ObstacleRect(0, 100+margin, 1830, 2550)

    # Rack palets
    rackCampHome = ObstacleRect(1175-margin, 1325+margin, 0, 102+margin)
    rackCampAway = ObstacleRect(1175-margin, 1325+margin, 2898-margin, 3000)
    rackGalerieHome = ObstacleRect(0, 102+margin, 1275, 1425)
    rackGalerieAway = ObstacleRect(0, 102+margin, 1575, 1725)

    # Sites de fouille
    digSiteHome = ObstacleRect(1200-margin, 1550+margin, 800-margin, 1150+margin)
    digSiteAway = ObstacleRect(1200-margin, 1550+margin, 1850-margin, 2200+margin)

    # Statuette piedestal
    statueHome = ObstacleTria((2000,   0), (1490-margin,   0), (2000, 510+margin))
    statueAway = ObstacleTria((2000,3000), (1490-margin,3000), (2000,2490-margin))

    # Walls 
    wallNorth = ObstacleRect(0, margin, 0, 3000)
    wallSouth = ObstacleRect(2000-margin, 2000, 510, 2490)
    wallEast = ObstacleRect(0, 2000, 0, margin)
    wallWest = ObstacleRect(0, 2000, 3000-margin, 3000)

    # Bases
    baseHome = ObstacleRect(400-margin, 1000+margin, 0, 400+margin)
    baseAway = ObstacleRect(400-margin, 1000+margin, 2600-margin, 3000)

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
        obstacleList.extend([rackCampHome, rackGalerieHome, rackGalerieAway])
        obstacleList.extend([galerieHome, galerieAway, statueHome, statueAway])
        obstacleList.extend([digSiteHome,digSiteAway])
        #obstacleList.append(baseHome)
    else:
        obstacleList.extend([wallNorth, wallEast, wallWest, wallSouth])
        obstacleList.extend([rackCampAway, rackGalerieHome, rackGalerieAway])
        obstacleList.extend([galerieHome, galerieAway, statueHome, statueAway])
        obstacleList.extend([digSiteHome,digSiteAway])
        #obstacleList.append(baseAway)

    LOG_INFO("Number of static obstacles : {}.".format(len(obstacleList)))
    return obstacleList
