#!/usr/bin/env python
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
@file: lidar_lib.py
@status: OK

Librairie de fonctions permettant le traitement des données reçues par le LiDAR.
"""

#######################################################################
# IMPORTS
#######################################################################

from __future__ import division

import math
import numpy as np
import os, sys
import rospy
import configparser

#######################################################################
# CONSTANTS
#######################################################################

TABLE_H = 3000          # hauteur de table (selon y)
TABLE_W = 2000          # largeur de table (selon x)

LOCAL_LIM = 100         # distance lim de regroupement/localisation

READER = configparser.ConfigParser()
READER.read(os.path.join(os.path.dirname(__file__),"../../../gr_config.ini"))

ROBOT_NAME = READER.get("ROBOT", "robot_name")
ROBOT_LARG = int(READER.get("ROBOT", "robot_larg"))
ROBOT_LONG = int(READER.get("ROBOT", "robot_long"))
ROBOT_DIAG = np.linalg.norm([ROBOT_LARG/2, ROBOT_LONG/2])
TABLE_MARGIN = ROBOT_DIAG/2 + 20      # marge aux bords de table



#######################################################################
# FUNCTIONS
#######################################################################

def LOG_INFO(msg):
    rospy.loginfo("[LID] :" + msg)

def handler(rcv_sig, frame):
	"""Forcer le nœud à quitter sur SIGINT, éviter d'aller jusqu'à le SIGTERM."""
	LOG_INFO("ISB Node forced to terminate...")
	rospy.signal_shutdown(rcv_sig)
	sys.exit()

# Calcule les coordonnées absolues d'un set de points du LiDAR dans le repère de la carte.
def lidar_to_table(x_r, y_r, cap, ranges, angle_min, angle_max, angle_inc, range_min, range_max):

    # Parametres de calcul
    margin = TABLE_MARGIN
    theta = angle_min
    
    # Liste des coordonnées des obstacles sur la table
    obstList = [] 
    for dist in ranges:
        # On applique un masque pour supprimer les points qui ne sont pas dans les bornes indiquées (bornes de détection du LiDAR).
        if range_min<dist<range_max:
            # Calcul des coords absolue sur la carte
            x_obs = x_r + 1000*dist*np.cos(cap + theta)
            y_obs = y_r + 1000*dist*np.sin(cap + theta)
            # On applique un deuxième masque une fois les coordonnées obtenues pour supprimer les points en dehors de la map.
            if (margin < y_obs < TABLE_H-margin) and (margin < x_obs < TABLE_W-margin):
                obstList.append([x_obs, y_obs])
        theta += angle_inc
    return obstList
    

def euclidean_distance(pt1, pt2):
    """Retourne la distance entre 2 points dans un repère cartésien. Ici, le repère de la table.    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
 

# Fonction permettant de clusteriser les points obtenus après traitement. Cela diminue le nombre de points et identifie les obstacles.
# Les points sont, par construction, dans le sens de l'augmentation de l'angle theta du LiDAR. Donc si on identifie un regroupement, le traitement effectué ci-dessous est valable dans le sens où l'on ne retrouvera pas de points dans ce même regroupement.
def clusterisation(obstList):

    opponentList = []

    if obstList == []: # Si pas d'obstacles
        return []
    
    else :  # Si obstacles
        lastPos = obstList[0]
        xBary = 0
        yBary = 0
        nbPts = 0

        for pos in obstList:
            # On regarde si le point suivant fait partie du regroupement en regardant sa distance au regroupement.
            if euclidean_distance(lastPos,pos) < LOCAL_LIM:
                nbPts += 1
                xBary += pos[0]
                yBary += pos[1]
                lastPos = pos
                continue
            
            # Sinon, s'il y a plus d'un point on moyennise pour obtenir des coordonnées barycentriques.
            if nbPts >= 1:
                xBary /= nbPts
                yBary /= nbPts 
                opponentList.append([xBary, yBary])
            # Et on cree un nouvel obstacle avec le point
            nbPts = 1
            xBary = pos[0]
            yBary = pos[1]
            lastPos = pos
        
        if nbPts >= 1:
            xBary /= nbPts
            yBary /= nbPts 
            opponentList.append([xBary, yBary])
        return opponentList
        

def need_stop():
    pass


