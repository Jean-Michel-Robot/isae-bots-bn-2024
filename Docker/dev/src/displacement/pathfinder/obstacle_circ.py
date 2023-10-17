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
@file: obstacle_circ.py
@status: OK.
"""

import math

class ObstacleCirc:
    
    """Implemente un obstacle circulaire sur la map."""

    def __init__(self, x_center, y_center, radius):
        """Initialization of obstacle."""
        self.name = "C"         # Type d'obstacle
        self.x_center = x_center  # Coordonnée selon l'axe X du centre du cercle
        self.y_center = y_center  # Coordonnée selon l'axe Y du centre du cercle
        self.radius = radius    # Rayon du cercle
        
    def get_x_center(self):
        return self.x_center
    
    def get_y_center(self):
        return self.y_center
    
    def set_x_center(self, x):
        self.x_center = x

    def set_y_center(self, y):
        self.y_center = y

    def get_radius(self):
        return self.radius
    
    def get_name(self):
        return self.name

    def is_node_in(self, node):
        """Verifie si le node passe en param est dans l'obstacle."""
        x = node.get_x()
        y = node.get_y()
        return math.sqrt((x-self.x_center)**2+(y-self.y_center)**2)<self.radius

    def is_equals(self, obstacle):
        return (self.x_center == obstacle.x_center) and (self.y_center == obstacle.y_center) and (self.radius == obstacle.radius)
    
    def copy(self):
        return ObstacleCirc(self.x_center, self.y_center, self.radius)