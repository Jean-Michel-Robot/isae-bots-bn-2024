#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: maps.py
@status: OK.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import os
from ast import literal_eval

from disp_utils import READER

#################################################################################################
if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else:
    from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################

class Maps:

    """ Classe représentant le terrain de jeu """

#######################################################################
# Methods
#######################################################################

    def __init__(self, standardNodeList, avoidingNodeList, obstacleList):
        """Initialization of Maps."""
        self.robotWidth = int(literal_eval(READER.get("Robot", "robot_larg")))

        self.obstacleList = obstacleList            # Liste des obstacles
        self.standardNodeList = standardNodeList    # Liste des noeuds de passages présents sur la Map        
        self.avoidingNodeList = avoidingNodeList    # Liste des noeud à utiliser lors de l'évitement

        self.obstacleRobotPos = None                # Position robot évitement

        # Choix map classique ou map d'évitement    #### ATRANSFORMER EN ENTIER POUR AVOIR PLUS QUE 2 MAPS ####
        self.avoid = False
        # Deuxieme essai evitement
        self.isSecondAttempt = False

    def getObstacleList(self):
        obstacleList = [o for o in self.obstacleList]

        if self.avoid:
            if self.isSecondAttempt:
                obstacleList.pop()
                return obstacleList+[self.obstacleRobotPos]
            else:
                return obstacleList+[self.obstacleRobotPos]
        else:
            return obstacleList
    
    def getNodeList(self):
        if self.avoid:
            return self.avoidingNodeList
        else:
            return self.standardNodeList

    def setObstacleRobotPos(self, obstacleRobotPos):
        self.obstacleRobotPos = obstacleRobotPos

    def setAvoid(self, avoid, isSecondAttempt):
        self.avoid = avoid
        self.isSecondAttempt = isSecondAttempt

    def getAvoid(self):
        return self.avoid

    # def setRack(self, rack):
    #     self.rack=rack

    # def getRack(self):
    #     return self.rack

    # def removeBouee(self, id):
    #     self.obstacleList.pop(id)

    # def addBouee(self, x, y):
    #     self.obstacleList.append(CircleObstacle(x, y, self.R_bouee + self.robotWidth/2 + self.boueeMargin))

    # def addDroppedBouees(self, color, side):
    #     if side == "left":
    #         if color == 0:
    #             self.obstacleList.append(RectangleObstacle(470, 540, 0, 400))
    #             self.obstacleList.append(CircleObstacle(505, 400, self.robotWidth/2 + self.boueeMargin))
    #             print("Added left blue rack obstacle")
    #         else: 
    #             self.obstacleList.append(RectangleObstacle(470, 540, 2600, 3000))
    #             self.obstacleList.append(CircleObstacle(505, 2600, self.robotWidth/2 + self.boueeMargin))
    #             print("Added left yellow rack obstacle")

    #     elif side == "right":
    #         if color == 0:
    #             self.obstacleList.append(RectangleObstacle(1040, 1110, 0, 400))
    #             self.obstacleList.append(CircleObstacle(1075, 400, self.robotWidth/2 + self.boueeMargin))
    #             print("Added right blue rack obstacle")
    #         else: 
    #             self.obstacleList.append(RectangleObstacle(470, 540, 2600, 3000))
    #             self.obstacleList.append(CircleObstacle(1075, 2600, self.robotWidth/2 + self.boueeMargin))
    #             print("Added right yellow rack obstacle")
    #     else: raise ValueError