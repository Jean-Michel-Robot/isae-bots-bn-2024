#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: obstacle_tria.py
@status: OK
"""

class ObstacleTria:
    
    """
    Implemente un obstacle triangle rectangle sur la map.
    
    L'obstacle est represente par : 
    - (x,y) : la position du sommet rectangle
    - b     : la base (negative si vers la gauche)
    - h     : la hauteur (negative si vers le bas).
    """

    def __init__(self,p1,p2,p3):
        """Initialization of obstacle."""       
        self.name = "T"    # Type d'obstacle - triangle rectangle
        self.p1 = p1        # point 1 sous la forme (x1,y1)
        self.p2 = p2        # point 2 sous la forme (x2,y2)
        self.p3 = p3        # point 3 sous la forme (x3,y3)
        
    def getP1(self):
        return self.p1
    
    def getP2(self):
        return self.p2
    
    def getP3(self):
        return self.p3
    
    def getName(self):
        return self.name

    def isNodeIn(self, node):
        x = node.getX()
        y = node.getY()

        ## Calcul des produits vectoriels 
        a = (self.p2[0]-x)*(self.p3[1]-y) - (self.p2[1]-y)*(self.p3[0]-x)
        b = (self.p3[0]-x)*(self.p1[1]-y) - (self.p3[1]-y)*(self.p1[0]-x)
        c = (self.p1[0]-x)*(self.p2[1]-y) - (self.p1[1]-y)*(self.p2[0]-x)

        ## On verifie que les angles ont le meme signe
        if a*b >= 0 and a*c >= 0 and b*c >= 0:
            return True
        return False

    