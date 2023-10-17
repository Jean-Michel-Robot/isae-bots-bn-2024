#!/usr/bin/env python3
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

"""Generates a grid for a map of the table."""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import xml.etree.ElementTree as ET
from xml.dom import minidom
from disp_utils import READER
import numpy as np

# On donne un nom au fichier regroupant tous les noeuds générés.
ofile = "./WholeTableGrid_away.xml"

# On initialise les paramètres pour la génération des points de la table
robot_width = int(READER.get("Robot", "robot_larg"))
robot_length = int(READER.get("Robot", "robot_long"))
robot_diag = np.linalg.norm([robot_width/2, robot_length/2])
margin = robot_diag // 2 + 20            # Pour en prendre en compte les dimensions du robot et que le point central du robot puisse se balader dans des zones autorisées.
min_x, max_x = 0+margin, 2000-margin     # Dimensions en x de la table.
min_y, max_y = 1500+margin, 3000-margin  # Dimensions en y de la table.

dx, dy = max_x-min_x, max_y-min_y        # Zone de déplacement autorisé
nb_points_x = 20                         # nb de points selon x
nb_points_y = int(nb_points_x * dy // dx)     # nb de points selon y
x_step = dx / (nb_points_x-1)            # ecart entre les points en x
y_step = dy / (nb_points_y-1)            # ecart entre les points en y

# Création de la grille à partir de ces données
grid = ET.Element("map")
counter = 0
for xId in range(nb_points_x):
    for yId in range(nb_points_y):
        node = ET.SubElement(grid, "node")
        node.set("id","p{}".format(counter))
        node.set("x","{}".format(min_x + int(xId * x_step)))
        node.set("y","{}".format(min_y + int(yId * y_step)))

        if(yId != nb_points_y-1): ## On connecte le point à celui devant lui (en y).
            link = ET.SubElement(grid, "connection")
            link.set("p1","p{}".format(counter))
            link.set("p2","p{}".format(counter+1))            
        if(xId != nb_points_x-1):
            connectionB = ET.SubElement(grid, "connection")
            connectionB.set("p1","p{}".format(counter))
            connectionB.set("p2","p{}".format(counter+nb_points_y))
        counter+=1

fid = open(ofile, "w")
xmlstr = minidom.parseString(ET.tostring(grid)).toprettyxml(indent="   ")
fid.write(xmlstr)
fid.close()
