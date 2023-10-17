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
""" if os.environ['USER'] == 'pi':
	from isae_robotics_msgs.msg import InfoMsg, ActionnersMsg, EndOfActionMsg 		# sur robot
else: """
from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg					# sur ordi
#################################################################################################
# from message.msg import InfoMsg, ActionnersMsg, EndOfActionMsg

class Maps:

    """ Classe représentant le terrain de jeu """

#######################################################################
# Methods
#######################################################################

    def __init__(self, standard_node_list, avoiding_node_list, obstacle_list):
        """Initialization of Maps."""
        self.robot_width = int(literal_eval(READER.get("ROBOT", "robot_larg")))

        self.obstacle_list = obstacle_list            # Liste des obstacles
        self.standard_node_list = standard_node_list    # Liste des noeuds de passages présents sur la Map        
        self.avoiding_node_list = avoiding_node_list    # Liste des noeud à utiliser lors de l'évitement

        self.obstacle_robot_pos = None                # Position robot à éviter

        # Choix map classique ou map d'évitement    #### ATRANSFORMER EN ENTIER POUR AVOIR PLUS QUE 2 MAPS ####
        self.avoid = False
        # Deuxieme essai evitement
        self.is_second_attempt = False

    def get_obstacle_list(self):
        if self.avoid:
            if self.is_second_attempt:
                return self.obstacle_list
            return self.obstacle_list+[self.obstacle_robot_pos]
        else:
            return self.obstacle_list
        
    def remove_obstacle(self, obstacle):
        for i in range(len(self.obstacle_list)):
            if type(self.obstacle_list[i]) == type(obstacle):
                if obstacle.is_equals(self.obstacle_list[i]):
                    del self.obstacle_list[i]
                    break
    
    def get_node_list(self):
        if self.avoid:
            return self.avoiding_node_list
        else:
            return self.standard_node_list

    def set_obstacle_robot_pos(self, obstacle_robot_pos):
        self.obstacle_robot_pos = obstacle_robot_pos
    
    def get_obstacle_robot_pos(self):
        return self.obstacle_robot_pos

    def set_avoid(self, avoid, is_second_attempt):
        self.avoid = avoid
        self.is_second_attempt = is_second_attempt

    def get_avoid(self):
        return self.avoid