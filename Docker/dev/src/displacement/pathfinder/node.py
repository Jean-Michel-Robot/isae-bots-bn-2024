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
@file: node.py
@status: in progress

Definition des noeuds de l'algo pour l'A*.
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
        self.link_node_list = []  # List des index des noeuds connectes
        
        self.goal_dist = None    # Distance à vol d'oiseau au noeud d'arrive
        self.init_dist = None    # Distance au noeud de départ (avec le chemin envisagé)

        self.parent = None      # Noeud parent       
        self.weight = 10000      # Poids du noeud dans l'algorithme A*
    
#######################################################################
# Getters & Setters
#######################################################################

    def get_position(self):
        return self.pos

    def get_x(self):
        return self.pos[0]
    
    def get_y(self):
        return self.pos[1]
    
    def get_weight(self):
        return self.weight

    def get_parent(self):
        return self.parent

    def get_init_dist(self):
        return self.init_dist

    def get_goal_dist(self):
        return self.goal_dist
        
    def get_link_node_list(self):
        """Gets the link node list of the node."""
        return self.link_node_list
        
    def set_parent(self, parent):
        self.parent = parent

    def set_weight(self, weight):
        self.weight = weight 
        
    def set_goal_dist(self, goal_dist):
        self.goal_dist = goal_dist
        """ if self.init_dist != None:
            self.weight = self.init_dist+self.goal_dist """
        
    def set_init_dist(self, startDist):
        self.init_dist = startDist
        """ if self.goal_dist != None: 
            self.weight = self.init_dist+self.goal_dist """

    def add_link_node_list(self,node):
        """Adds a node to the linkNodeList."""
        self.link_node_list.append(node)

#######################################################################
# Modification du noeud parent ainsi que des attributs du noeud actuel
####################################################################### 

    def dist_from_node(self, node):
        '''Calcul la distance euclidienne avec le noeud passé en paramètre.'''
        return math.sqrt((node.get_x()-self.get_x())**2+(node.get_y()-self.get_y())**2)

    def calcul_weight(self, parent):
        """Calcul du nouveau poids à partir d'un noeud parent (pour voir si l'on doit changer le noeud parent ou non)."""
        return self.goal_dist + parent.get_init_dist() + self.dist_from_node(parent)   
    
    def update_node(self, parent):
        '''Fonction de mise à jour du noeud (changement de parent).'''
        self.init_dist = parent.get_init_dist() + self.dist_from_node(parent)
        self.parent = parent
        self.weight = self.calcul_weight(parent)

#######################################################################
# Manipulation sur les noeuds
#######################################################################
    
    def equals(self, node):
        """Verifie l'egalite avec le noeud passe en parametre."""
        return node.get_x() == self.get_x() and node.get_y() == self.get_y()
  
    def is_visible(self, node, tableMap): # TODO - change
        '''Vérifie si le noeud passé en paramètre est visible, i.e pas d'obstacle sur la route'''
        '''On peut se permettre par la suite de voir l'intersection du centre de notre robot avec les obstacles, car les dimensions du robot sont prise
        en compte dans la marge.'''

        if self.equals(node):
            return False
        
        node_dist = self.dist_from_node(node)
        line_vect = np.array([node.get_x()-self.get_x(), node.get_y()-self.get_y()])
        line_vect = line_vect / np.linalg.norm(line_vect)
        
        for obstacle in tableMap.get_obstacle_list():
            if obstacle.get_name() == "C":
                x = obstacle.get_x_center()
                y = obstacle.get_y_center()
                r = obstacle.get_radius()
                
                ## det4 correspond au déterminant de l'équation caractéristique du second ordre calculant la ou les intersections avec le cercle étudié.
                ## Si le déterminant est >= 0, il existe une intersection, et on regarde alors si cette intersection se trouve sur le chemin pour aller au noeud.
                det4 = ((self.get_x()-x)*line_vect[0]+(self.get_y()-y)*line_vect[1])**2-((line_vect[0]**2+line_vect[1]**2)*((self.get_x()-x)**2)+(self.get_y()-y)**2-r**2)
                if det4 >= 0:
                    tp = -((self.get_x()-x)*line_vect[0]+(self.get_y()-y)*line_vect[1]+math.sqrt(det4))/(line_vect[0]**2+line_vect[1]**2)
                    tm = -((self.get_x()-x)*line_vect[0]+(self.get_y()-y)*line_vect[1]-math.sqrt(det4))/(line_vect[0]**2+line_vect[1]**2)
                    if 0 < tp < node_dist or 0 < tm < node_dist:
                        return False
                    
            elif obstacle.get_name() == "T":
                ## Pas d'obstacles triangulaires pour le moment.
                continue

            elif obstacle.get_name() == "R":
                x_min = obstacle.get_x_min()
                x_max = obstacle.get_x_max()
                y_min = obstacle.get_y_min()
                y_max = obstacle.get_y_max()
                
                if line_vect[0] != 0 :
                    t_min = (x_min-self.get_x())/line_vect[0]
                    t_max = (x_max-self.get_x())/line_vect[0]

                    if (y_min<self.get_y()+t_min*line_vect[1]<y_max and 0<t_min<node_dist) or (y_min<self.get_y()+t_max*line_vect[1]<y_max and 0<t_max<node_dist):

                        return False
    
                if line_vect[1] != 0 :
                    t_min = (y_min-self.get_y())/line_vect[1]
                    t_max = (y_max-self.get_y())/line_vect[1]
                    
                    if (x_min<self.get_x()+t_min*line_vect[0]<x_max and 0<t_min<node_dist ) or (x_min<self.get_x()+t_max*line_vect[0]<x_max and 0<t_max<node_dist):

                        return False
            
            else:
                raise TypeError("Obstacle type unknown... Obstacle.get_name() = {}".format(obstacle.get_name()))
                
        return True
    
        
        
        
    
        
    
        
    

    
    