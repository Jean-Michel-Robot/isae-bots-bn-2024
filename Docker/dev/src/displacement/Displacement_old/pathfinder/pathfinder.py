#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: pathfinder.py
@status: in progress.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

from maps import Maps
from astar import a_star
from exceptions import PathNotFoundError

import nodes_creator as nc
import obstacles_creator as oc

from obstacle_rect import ObstacleRect
from obstacle_circ import ObstacleCirc

from disp_utils import READER, LOG_INFO
from ast import literal_eval

#######################################################################
#
#                           Class PATHFINDER
#
#######################################################################

class Pathfinder:
    
    """Pathfinder class."""

    def __init__(self, color):
        """Initialization of Pathfinder."""
        
        avoid = 2
        self.tableMap = Maps(nc.makeNodeList(color),nc.makeNodeList(avoid),oc.makeObstacleList(color))
        self.init_pos = None
        self.goal_pos = None
        self.robotToAvoidPos = None        
        
        self.color = color   # Color : 0 Home | 1 Away | 
        self.maxAstarTime = int(literal_eval(READER.get("Pathfinder", "max_astar_time")))
        
    def setInit(self, pos):
        self.init_pos = pos
        
    def setGoal(self,pos):
        self.goal_pos = pos

    def setRobotToAvoidPos(self, pos, radius):
        self.robotToAvoidPos = [pos, radius]

    def getRobotToAvoidPos(self):
        return self.robotToAvoidPos
    
    def getTableMap(self):
        return self.tableMap

    def setMaxAstarTime(self, time):
        self.maxAstarTime = time

#######################################################################
#                            COMPUTE PATH 
#######################################################################
        
    def getPath(self, isAvoid, isFirstAccurate, isSecondAttempt):
        if isAvoid:
            if self.robotToAvoidPos is None:
                LOG_INFO("ERREUR : IL N'Y A PAS EU DE SETAVOIDROBOT.")
                raise PathNotFoundError
            self.tableMap.setObstacleRobotPos(ObstacleCirc(self.robotToAvoidPos[0][0], self.robotToAvoidPos[0][1], self.robotToAvoidPos[1]))
            self.tableMap.setAvoid(True, isSecondAttempt)
        else:
            self.tableMap.setAvoid(False, False)
            
        return a_star(self.init_pos, self.goal_pos, self.tableMap, isFirstAccurate, self.maxAstarTime)
