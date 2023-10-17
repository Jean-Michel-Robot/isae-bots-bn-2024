#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: obstacle_rect.py
@status: OK.
"""

class ObstacleRect:
    
    """Implemente un obstacle rectangulaire sur la map."""

    def __init__(self,xMin,xMax,yMin,yMax):
        """Initialization of obstacle."""    
        self.name = "R"     # Type d'obstacle
        self.xMin = xMin    # xMin du rectangle
        self.xMax = xMax    # xMax du rectangle
        self.yMin = yMin    # yMin du rectangle
        self.yMax = yMax    # yMax du rectangle
        
    def getXMin(self):
        return self.xMin
    
    def getXMax(self):
        return self.xMax
    
    def getYMin(self):
        return self.yMin
    
    def getYMax(self):
        return self.yMax
    
    def getName(self):
        return self.name

    def isNodeIn(self, node):
        x = node.getX()
        y = node.getY()
        return self.xMin<x<self.xMax and self.yMin<y<self.yMax

    