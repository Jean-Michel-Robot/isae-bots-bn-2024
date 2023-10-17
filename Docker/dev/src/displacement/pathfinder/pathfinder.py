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
@file: pathfinder.py
@status: in progress.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

from pathfinder.maps import Maps
from pathfinder.astar import a_star
from pathfinder.exceptions import PathNotFoundError

import pathfinder.nodes_creator as nc
import pathfinder.obstacles_creator as oc
from pathfinder.obstacle_rect import ObstacleRect
from pathfinder.obstacle_circ import ObstacleCirc


# from .disp_utils import *
from disp_utils import READER, log_info
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
        self.table_map = Maps(nc.make_node_list(color),nc.make_node_list(avoid),oc.make_obstacle_list(color))
        self.init_pos = None
        self.goal_pos = None
        self.robot_to_avoid_pos = None        
        
        self.color = color   # Color : 0 Home | 1 Away | 
        self.max_astar_time = int(literal_eval(READER.get("PATHFINDER", "max_astar_time")))
        
    def set_init(self, pos):
        self.init_pos = pos
        
    def set_goal(self,pos):
        self.goal_pos = pos

    def set_robot_to_avoid_pos(self, pos, radius):
        self.robot_to_avoid_pos = [pos, radius]
        self.table_map.set_obstacle_robot_pos(ObstacleCirc(self.robot_to_avoid_pos[0][0], self.robot_to_avoid_pos[0][1], self.robot_to_avoid_pos[1]))


    def get_robot_to_avoid_pos(self):
        return self.robot_to_avoid_pos
    
    def get_table_map(self):
        return self.table_map

    def remove_obstacle(self, obstacle):
        #log_info("DELETE LES OBST")
        self.table_map.remove_obstacle(obstacle)

    def set_max_astar_time(self, time):
        self.max_astar_time = time

#######################################################################
#                            COMPUTE PATH 
#######################################################################
        
    def get_path(self, isAvoid, isFirstAccurate, isSecondAttempt):
        if isAvoid:
            if self.robot_to_avoid_pos is None:
                self.set_robot_to_avoid_pos([-1000, -1000], 0)
            self.table_map.set_obstacle_robot_pos(ObstacleCirc(self.robot_to_avoid_pos[0][0], self.robot_to_avoid_pos[0][1], self.robot_to_avoid_pos[1]))
            self.table_map.set_avoid(True, isSecondAttempt)
        else:
            self.table_map.set_avoid(False, False)
            
        #print("POSITION ADV " + str(self.table_map.get_obstacle_robot_pos().get_x_center()))
        return a_star(self.init_pos, self.goal_pos, self.table_map, isFirstAccurate, self.max_astar_time)
