#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: obstacle_circ.py
@status: OK.
"""

import math

class ObstacleCirc:
    
    """Implemente un obstacle circulaire sur la map."""

    def __init__(self, xCenter, yCenter, radius):
        """Initialization of obstacle."""
        self.name = "C"         # Type d'obstacle
        self.xCenter = xCenter  # Coordonnée selon l'axe X du centre du cercle
        self.yCenter = yCenter  # Coordonnée selon l'axe Y du centre du cercle
        self.radius = radius    # Rayon du cercle
        
    def getXCenter(self):
        return self.xCenter
    
    def getYCenter(self):
        return self.yCenter
    
    def getRadius(self):
        return self.radius
    
    def getName(self):
        return self.name

    def isNodeIn(self, node):
        """Verifie si le node passe en param est dans l'obstacle."""
        x = node.getX()
        y = node.getY()
        return math.sqrt((x-self.xCenter)**2+(y-self.yCenter)**2)<self.radius