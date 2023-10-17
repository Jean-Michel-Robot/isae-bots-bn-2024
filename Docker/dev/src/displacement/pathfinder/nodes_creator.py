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
@file: nodes_creator.py
@status: OK
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import os

import xml.etree.ElementTree as ET
from pathfinder.node import Node

from disp_utils import *

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def make_node_list(option):
    """
    Function to create a list of nodes for the pathfinder grid of nodes.
    
    Parameters 
    ----------
    option: [int] | -1, 0, ..., n 
        Specifies the type of map that must be used (avoiding, config1, config2, ...).
        In the case of the years before 2023, there were 2 config : HOME and AWAY.
        In the case of the year 2023, there were multiple config depending of the start position (but always two colors).
    
    Return
    ------
    nodelist: [list(Node)]
        A list of Node objects - the node list for the map.
    """
  
    ## CONFIG DATA ####################################################
    if ROBOT_NAME == "\"GR\"":
        data0 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))
        data1 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))
        data2 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_avoiding.xml"))
    else:
        data0 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_home.xml"))
        data1 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_away.xml"))
        data2 = ET.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))

    if option == 0:
        data = data0
    elif option == 1:
        data = data1
    elif option == 2:
        data = data2
    else:
        raise RuntimeError("Specified option is invalid")

    ## BUILDING NODES #################################################
    nodeList = []
    for nodeParam in data.findall("node"):
        nodeList.append(Node([int(nodeParam.get("x")),int(nodeParam.get("y"))])) 
    for linkParam in data.findall("connection"):
        node1Index = int(linkParam.get("p1")[1:])
        node2Index = int(linkParam.get("p2")[1:])
        nodeList[node1Index].add_link_node_list(nodeList[node2Index])
        nodeList[node2Index].add_link_node_list(nodeList[node1Index])
    return nodeList
