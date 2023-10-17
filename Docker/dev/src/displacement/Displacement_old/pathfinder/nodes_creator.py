#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

from lxml import etree
from node import Node

from disp_utils import ROBOT_NAME

#######################################################################
#
#                             DEFINITIONS
#
#######################################################################

def makeNodeList(option):
    """
    Function to create a list of nodes for the pathfinder grid of nodes.
    
    Parameters 
    ----------
    option: [int] | 0, 1 or 2
        Specifies the type of map must be used (home, away, avoiding).
    
    Return
    ------
    nodelist: [list(Node)]
        A list of Node objects - the node list for the map.
    """
  
    ## CONFIG DATA ####################################################
    if ROBOT_NAME == "PR":
        data0 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))
        data1 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))
        data2 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_avoiding.xml"))
    else:
        data0 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_home.xml"))
        data1 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_away.xml"))
        data2 = etree.parse(os.path.join(os.path.dirname(__file__),"../pathfinder_data/match_grid_complete.xml"))

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
    for nodeParam in data.xpath("/map/node"):
        nodeList.append(Node([int(nodeParam.get("x")),int(nodeParam.get("y"))])) 
    for linkParam in data.xpath("/map/connection"):
        node1Index = int(linkParam.get("p1")[1:])
        node2Index = int(linkParam.get("p2")[1:])
        nodeList[node1Index].addLinkNodeList(nodeList[node2Index])
        nodeList[node2Index].addLinkNodeList(nodeList[node1Index])
    return nodeList
