#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false
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


import os
import roslib
import rospy

import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension



def log_info(msg):
     rospy.loginfo("[SEN] "+ msg)

def log_err(msg):
    rospy.logerr("[SEN] "+ msg)

# pour le test#
def lineAngle(x0, y0, x1, y1):

	if x1 == x0:
		if y1 == y0:
			log_err("ERREUR : memes coordonnes pour les deux points de la line")
		theta = np.sign(y1 - y0)*np.pi/2

	elif y1 == y0:
		theta = (1 - np.sign(x1 - x0))*np.pi/2

	else:
		theta = np.arctan((y1 - y0)/(x1 - x0))

	if x0>x1:  # cas ou l'arctan vaut en fait theta +- pi
		if y0<y1:
			theta = theta + np.pi
		else:
			theta = theta - np.pi
	return theta



class SensorsNode:
    DISTANCEMIN = 100
    INTERVALLE = 1
    LIMITEBASSE = 10
    obstaclesLidar = []
    obstaclesSonars = []
    countLidar = 1
    countSonars = 1
    x_robot=0
    y_robot=0
    cap = 0

    def update_position(self,msg):
        """Fonction de callback de position."""
        
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.c_robot = msg.theta

        '''
        ####
        x = 800
        y = 1200
        d = (int)(np.linalg.norm([self.x_robot - x, self.y_robot- y]))
        theta = lineAngle(self.x_robot, self.y_robot, x, y)
        dx = (int)(abs(d*np.cos(theta)))
        dy = (int)(abs(d*np.sin(theta)))
        newmsg = Int16MultiArray()

        #newmsg.data = [0, [1000, 1300, 300, 300]]
        newmsg.data = [0, x, y, d, dx, dy]

        layout = MultiArrayLayout()
        layout.data_offset = 1

        dimensions = []
        dim1 = MultiArrayDimension()
        dim1.label = "obstacle_nb"
        dim1.size = 1
        dim1.stride = 5*1

        dim2 = MultiArrayDimension()
        dim2.label = "info"
        dim2.size = 5
        dim2.stride = 5

        dimensions.append(dim1)
        dimensions.append(dim2)

        layout.dim = dimensions

        newmsg.layout = layout

        self.pub_obstaclesInfo.publish(newmsg)
        ####
        '''




	#fonction callback sur retour d'obstacles
    #ébauche de traitement des données LIDAR et SONAR pour déterminer des vitesses/directions. Ce code n'est pas efficace
    def update_obstacles(self, msg):
        #le premier indice de la liste vaut 0 si ce sont des obstacles LIDAR, 1 si SONAR
        if msg.data[0] == 0:
            count = self.countLidar
            obstacles = self.obstaclesLidar
            self.countLidar = (self.countLidar + 1) % self.INTERVALLE
        elif msg.data[0] == 1:
            count = self.countSonars
            obstacles = self.obstaclesSonars
            self.countSonars = (self.countSonars + 1) % self.INTERVALLE
        if count==0:
            infoList =[]
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
                    if np.linalg.norm( np.array(nearestObstacle) - np.array(newObstacle)) < self.DISTANCEMIN :
                        dx = newObstacle[0] - nearestObstacle[0]
                        dy = newObstacle[1] - nearestObstacle[1]
                        if np.linalg.norm([dx,dy]) < self.LIMITEBASSE:
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

         
	
    def __init__(self):
        
        rospy.init_node('SensorsNode')
        log_info("Initializing SensorsNode ...")

        self.pub_obstaclesInfo=rospy.Publisher("/obstaclesInfo", Int16MultiArray, queue_size=10, latch=False)

        # initialisation des suscribers
        self.subLidar=rospy.Subscriber("/sensors/obstaclesLidar", Int16MultiArray, self.update_obstacles)
        self.subSonars = rospy.Subscriber("/sensors/obstaclesSonar", Int16MultiArray, self.update_obstacles)
        self.sub_rospy=rospy.Subscriber("/current_position", Pose2D, self.update_position)

    
if __name__ == '__main__':
    node = SensorsNode()
    rospy.spin()