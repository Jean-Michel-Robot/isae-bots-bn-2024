#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file: node.py
@status: in progress

Definition des noeuds de l'algo A star.
"""

import math
import numpy as np

#######################################################################
#
#                               Class NODE
#
#######################################################################

class Node:

    """Classe Node."""

    def __init__(self, pos):
        """Initialize a node."""
        self.pos = pos          # Position du noeud sur la table
        self.linkNodeList = []  # List des index des noeuds connectes
        
        self.goalDist = None    # Distance à vol d'oiseau au noeud d'arrive
        self.initDist = None    # Distance au noeud de départ (avec le chemin envisagé)

        self.parent = None      # Noeud parent       
        self.weight = None      # Poids du noeud dans l'algorithme A*
    
#######################################################################
# Parametrisation of Node
#######################################################################

    def getPosition(self):
        return self.pos

    def getX(self):
        return self.pos[0]
    
    def getY(self):
        return self.pos[1]
    
    def getWeight(self):
        return self.weight
        
    def addLinkNodeList(self,node):
        """Adds a node to the linkNodeList."""
        self.linkNodeList.append(node)
        
    def getLinkNodeList(self):
        """Gets the link node list of the node."""
        return self.linkNodeList
        
    def setParent(self, parent):
        self.parent = parent

    def getParent(self):
        return self.parent
        
    def setGoalDist(self, goalDist):
        self.goalDist = goalDist
        if self.initDist != None:
            self.weight = self.initDist+self.goalDist
        
    def setInitDist(self, startDist):
        self.initDist = startDist
        if self.goalDist != None: 
            self.weight = self.initDist+self.goalDist
        
    def getInitDist(self):
        return self.initDist

#######################################################################
# Manipulation in computations
#######################################################################
    
    def equals(self, node):
        """Verifie l'egalite avec le noeud passe en parametre."""
        return np.array_equal(node.getPosition(), self.getPosition())
  
    def isVisible(self, node, tableMap): # TODO - change
        '''Vérifie si le noeud passé en paramètre est visible.'''

        if self.equals(node):
            return False
        
        nodeDist = self.distFromNode(node)
        lineVect = np.array([node.getX()-self.getX(), node.getY()-self.getY()])
        lineVect = lineVect / np.linalg.norm(lineVect)
        
        for obstacle in tableMap.getObstacleList():
            if obstacle.getName() == "C":
                x = obstacle.getXCenter()
                y = obstacle.getYCenter()
                r = obstacle.getRadius()
                
                det4 = ((self.getX()-x)*lineVect[0]+(self.getY()-y)*lineVect[1])**2-((lineVect[0]**2+lineVect[1]**2)*((self.getX()-x)**2)+(self.getY()-y)**2-r**2)
                if det4 >= 0:
                    tp = -((self.getX()-x)*lineVect[0]+(self.getY()-y)*lineVect[1]+math.sqrt(det4))/(lineVect[0]**2+lineVect[1]**2)
                    tm = -((self.getX()-x)*lineVect[0]+(self.getY()-y)*lineVect[1]-math.sqrt(det4))/(lineVect[0]**2+lineVect[1]**2)
                    if 0 < tp < nodeDist or 0 < tm < nodeDist:
                        return False
                    
            elif obstacle.getName() == "T":
                continue

            elif obstacle.getName() == "R":
                xMin = obstacle.getXMin()
                xMax = obstacle.getXMax()
                yMin = obstacle.getYMin()
                yMax = obstacle.getYMax()
                
                if lineVect[0] != 0 :
                    tMin = (xMin-self.getX())/lineVect[0]
                    tMax = (xMax-self.getX())/lineVect[0]

                    if (yMin<self.getY()+tMin*lineVect[1]<yMax and 0<tMin<nodeDist) or (yMin<self.getY()+tMax*lineVect[1]<yMax and 0<tMax<nodeDist):
                        return False
    
                if lineVect[1] != 0 :
                    tMin = (yMin-self.getY())/lineVect[1]
                    tMax = (yMax-self.getY())/lineVect[1]
                    
                    if (xMin<self.getX()+tMin*lineVect[0]<xMax and 0<tMin<nodeDist ) or (xMin<self.getX()+tMax*lineVect[0]<xMax and 0<tMax<nodeDist):
                        return False
            
            else:
                raise TypeError("obstacle type unknown... Obstacle.getName() = {}".format(obstacle.getName()))
                
        return True
        
#######################################################################
# UPDATE du node
####################################################################### 

    def distFromNode(self, node):
        '''Calcul la distance euclidienne avec le noeud passé en paramètre.'''
        return math.sqrt((node.getX()-self.getX())**2+(node.getY()-self.getY())**2)

    def calculWeight(self, parent):
        """Calcul du nouveau poids (si on change de noeud parent)."""
        return self.goalDist + parent.getInitDist() + self.distFromNode(parent)   
    
    def updateNode(self, parent):
        '''Fonction de mise à jour du noeud (changement de parent).'''
        self.initDist = parent.getInitDist() + self.distFromNode(parent)
        self.parent = parent
        self.weight = self.calculWeight(parent)
        
        
        
    
        
    
        
    

    
    