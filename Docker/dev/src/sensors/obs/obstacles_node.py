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

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import rospy
import numpy as np
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose2D

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[OBS]"

def log_errs(msg):
    rospy.logerr(f"{_NODENAME_} {msg}")

def log_info(msg):
    rospy.loginfo(f"{_NODENAME_} {msg}")


def line_angle(x0, y0, x1, y1):
    """
    Function for checking line angles
    """
    if x1 == x0:
        if y1 == y0: log_errs("Same coordinates of the two points given.")
        theta = np.sign(y1 - y0)*np.pi/2
    elif y1 == y0:
        theta = (1 - np.sign(x1 - x0))*np.pi/2
    else:
        theta = np.arctan((y1 - y0)/(x1 - x0))

    if x0 < x1:
        return theta
    # sinon cas ou l'arctan vaut en fait theta +- pi
    if y0 < y1:
        return theta + np.pi
    else:
        return theta - np.pi
    
#################################################################
#                                                               #
#                           OBS Node                            #
#                                                               #
#################################################################

class ObstaclesNode:
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

    def __init__(self):
        log_info("Initializing OBS node ...")

        self.x_robot = None
        self.y_robot = None
        self.c_robot = None
        self.lidar_obs_lst = []
        self.sonar_obs_lst = []
        self.nb_lidar = 1
        self.nb_sonar = 1

        self.obs_pub = rospy.Publisher("/obstaclesInfo", Int16MultiArray, queue_size=10, latch=False)
        self.lidar_sub = rospy.Subscriber("/sensors/obstaclesLidar", Int16MultiArray, self.update_obstacles)
        #self.sonar_sub = rospy.Subscriber("/sensors/obstaclesSonar", Int16MultiArray, self.update_obstacles)
        self.pos_sub = rospy.Subscriber("/current_position", Pose2D, self.recv_position)

    def recv_position(self, msg):
        """
        Feedback on current position via /disp/current_position topic.
        """
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.c_robot = msg.theta

    def recv_obstacle(self, msg):
        """
        Feedback on detected obstacles.
        """
        if msg.data[0] == 0:  # lidar obstacle
            nbr = self.nb_lidar
            obs = self.lidar_obs_lst
            self.nb_lidar = (self.nb_lidar + 1) % self.INTERVALLE




	#fonction callback sur retour d'obstacles
    #ébauche de traitement des données LIDAR et SONAR pour déterminer des vitesses/directions. Ce code n'est pas efficace
    def update_obstacles(self, msg):
        #le premier indice de la liste vaut 0 si ce sont des obstacles LIDAR, 1 si SONAR
        if msg.data[0] == 0:
            #rospy.loginfo('Par ici')
            count = self.countLidar
            obstacles = self.obstaclesLidar
            self.countLidar = (self.countLidar + 1) % self.INTERVALLE
        elif msg.data[0] == 1:
            #rospy.loginfo('Par la')
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

            self.obs_pub.publish(newmsg)


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rospy.init_node("OBS node")
    node = ObstaclesNode()
    rospy.spin()

if __name__ == '__main__':
    main()
