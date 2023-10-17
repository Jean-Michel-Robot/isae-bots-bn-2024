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
@file: obstacle_rect.py
@status: OK.
"""

class ObstacleRect:
    
    """Implemente un obstacle rectangulaire sur la map."""

    def __init__(self,x_min,x_max,y_min,y_max):
        """Initialization of obstacle."""    
        self.name = "R"     # Type d'obstacle
        self.x_min = x_min    # x_min du rectangle
        self.x_max = x_max    # x_max du rectangle
        self.y_min = y_min    # y_min du rectangle
        self.y_max = y_max    # y_max du rectangle
        
    def get_x_min(self):
        return self.x_min
    
    def get_x_max(self):
        return self.x_max
    
    def get_y_min(self):
        return self.y_min
    
    def get_y_max(self):
        return self.y_max
    
    def get_name(self):
        return self.name

    def is_node_in(self, node):
        x = node.get_x()
        y = node.get_y()
        return self.x_min<x<self.x_max and self.y_min<y<self.y_max

    