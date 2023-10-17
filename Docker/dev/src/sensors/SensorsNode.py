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


from __future__ import division


import os
import roslib
import rospy

import numpy as np
from math import atan2
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16MultiArray, Int16
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

from displacement.disp_utils import patch_frame_br, log_errs, log_info

#################################################################
if os.environ['USER'] == 'pi':
	SIMULATION = False
else:
	SIMULATION = True
#################################################################

### CONSTANTES ###

DISTANCEMIN = 100
INTERVALLE = 1
LIMITEBASSE = 10

COLOR = {
      0: 'HOME',
      1: 'AWAY'
}

# Fonction permettant de calculer l'angle entre 2 points dans un repère cartésien.
def lineAngle(x0, y0, x1, y1):

	return atan2((y1-y0)/(x1-x0))
    

class SensorsNode:

    def __init__(self):

        rospy.init_node('SensorsNode')
        rospy.loginfo("Initialisation du Traitement des Capteurs")

        # Attributs du noeuds

        self.obstaclesLidar = []
        self.obstaclesSonars = []
        self.countLidar = 1
        self.countSonars = 1
        self.x_robot=0
        self.y_robot=0
        self.cap = 0
        self.color = None
        
        # Publishers & Subscribers nécessaires pour le noeud

        self.pub_obstaclesInfo=rospy.Publisher("/obstaclesInfo", Int16MultiArray, queue_size=10, latch=False)

        self.subLidar=rospy.Subscriber("/lidar/obstaclesLidar", Int16MultiArray, self.update_obstacles)
        self.subSonars = rospy.Subscriber("/sonar/obstaclesSonar", Int16MultiArray, self.update_obstacles)
        self.sub_rospy=rospy.Subscriber("/current_position", Pose2D, self.update_position)
        self.sub_color = rospy.Subscriber('/game/color', Int16, self.update_color)

    def update_color(self, msg):
        """
        Callback function from topic /sm/color.
        """
        if msg.data not in [0,1]:
            log_errs(f"Wrong value of color given ({msg.data})...")
            return
        else: 
            self.color = msg.data
            log_info("Received color : {}".format(COLOR[self.color]))

    # Fonciton callback pour la position
    def update_position(self,msg):

        x,y,c = patch_frame_br(msg.x,msg.y,msg.theta)
        self.x_robot = x
        self.y_robot = y
        self.c_robot = c

	# Fonction callback sur retour des données obstacles LiDAR et Sonar + Traitement pour déterminer la position de ces obstacles.
    def update_obstacles(self, msg):

        count = 0
        obstacles = []

        if msg.data[0] == 0: # Données LiDAR
            count = self.countLidar
            obstacles = self.obstaclesLidar
            self.countLidar = (self.countLidar + 1) % INTERVALLE

        elif msg.data[0] == 1: # Données Sonar
            count = self.countSonars
            obstacles = self.obstaclesSonars
            self.countSonars = (self.countSonars + 1) % INTERVALLE

        if count == 0:
            infoList = []
            newObstacles = []
            for i in range((msg.layout.dim[0]).size):
                newObstacles.append([msg.data[2*i +1], msg.data[2*i + 2]])
            for newObstacle in newObstacles:
                dx = 0
                dy = 0   
                #on va déterminer les directions et vitesses (partie à améliorer)
                if (len(obstacles) != 0):                              #pour chaque obstacle on va chercher l'obstacle précédent le plus proche
                    nearestObstacle = obstacles[0]                                 #si ce dernier est assez proche, on suppose que c'est le même obstacle qui s'est déplacé
                    for obstacle in obstacles:
                        if np.linalg.norm(np.array(obstacle) - np.array(newObstacle)) < np.linalg.norm(np.array(nearestObstacle) -np.array(newObstacle)):
                            nearestObstacle = obstacle 
                    if np.linalg.norm( np.array(nearestObstacle) - np.array(newObstacle)) < DISTANCEMIN :
                        dx = newObstacle[0] - nearestObstacle[0]
                        dy = newObstacle[1] - nearestObstacle[1]
                        if np.linalg.norm([dx,dy]) < LIMITEBASSE:
                            dx = 0
                            dy = 0
                info = [newObstacle[0], newObstacle[1], ((int)(np.linalg.norm([self.x_robot - newObstacle[0], self.y_robot- newObstacle[1]]))), dx, dy]
                infoList.append(info)
                if msg.data[0] == 0:
                    self.obstaclesLidar = newObstacles
                elif msg.data[0] == 1:
                    self.obstaclesSonars = newObstacles

            #on renvoie toutes les caractéristiques calculées sur un autre topic

            newmsg = Int16MultiArray()

            data = [msg.data[0]]
            for infoObstacle in infoList:
                for info in infoObstacle:
                    data.append(info)

            newmsg.data = data

            layout = MultiArrayLayout()
            layout.data_offset = 1

            dimensions = []
            dim1 = MultiArrayDimension()
            dim1.label = "obstacle_nb"
            dim1.size = len(infoList)
            dim1.stride = 5* len(infoList)

            dim2 = MultiArrayDimension()
            dim2.label = "info"
            dim2.size = 5
            dim2.stride = 5

            dimensions.append(dim1)
            dimensions.append(dim2)

            layout.dim = dimensions

            newmsg.layout = layout

            self.pub_obstaclesInfo.publish(newmsg)

    
if __name__ == '__main__':
    node = SensorsNode()
    rospy.spin()