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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import time
import rospy
import numpy as np
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose2D

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[SIM/OBS]"

def oscillationY(ti, ymin, ymax, w, phi):
	return (ymin + ymax)/2 + (ymax - ymin)/2 * np.sin(w * (time.time() - ti) + phi)

def oscillationX(ti, xmin, xmax, w, phi):
	return (xmin + xmax)/2 + (xmax - xmin)/2 * np.sin(w * (time.time() - ti) + phi)

#################################################################
#                                                               #
#                        OBSTACLES NODE                         #
#                                                               #
#################################################################

def loginfo(msg):
    rospy.loginfo(f"{_NODENAME_} " + msg)

class SIM_ObstaclesNode:
    """
    SIM OBS node: obstacles ros node for simulation.
    """

    def __init__(self):
        loginfo("Initializing OBS node ...")

        self.position_sub = rospy.Subscriber("/current_position", Pose2D, self.recv_position)
        self.obs_info_pub = rospy.Publisher("/obstaclesInfo", Int16MultiArray, queue_size=10, latch=False)
        # self.obs_lidar_pub = rospy.Publisher("/lidar/obstaclesLidar", Int16MultiArray, queue_size=10, latch=False)

        self.curr_time = time.time()

    def seen_obstacle(self, nbr=0):
        """
        Generates positions for fake obstacles.
        """
        # NB: try and upgrade this function for more complex examples...
        # obs_positions = [(1468,oscillationY(self.ti, 1800, 2500, 1, 0)),(oscillationX(self.ti, 1000, 1500, 1, 0),1550)]
        obs_positions = [(1200, 600)]
        return obs_positions

    def recv_position(self, msg):
        """
        Feedback of obstacles positions for simulation
        """
        ###############################################################
        ## Info about the robot
        ###############################################################
        self.x_robot = msg.x
        self.y_robot = msg.y
        self.cap = msg.theta

        ###############################################################
        ## Make the info msg to send
        ###############################################################
        # calculatedObstacles = 
        if time.time() - self.curr_time <= 15:
            obstacles_pos = []  #[(1200,600)]
        else:
            obstacles_pos = [(1000,800+30*(time.time()-self.curr_time-15), np.linalg.norm([self.x_robot-1000, self.y_robot-800+30*(time.time()-self.curr_time-15)]) ,0,0)]
        """ else :
            obstacles_pos = [(1000,1200, np.linalg.norm([self.x_robot-1000, self.y_robot-1200]) ,0,0)] """
        #loginfo("Pos obst :" + str(obstacles_pos))

        #obstacles_pos = []

        data = [0]
        for pos in obstacles_pos:
            for coord in pos:
                data.append((int)(coord))

        dim1 = MultiArrayDimension()
        dim1.label = "nbObstacles"
        dim1.size = len(obstacles_pos)
        dim1.stride = 5* len(obstacles_pos)

        dim2 = MultiArrayDimension()
        dim2.label = "coordinates"
        dim2.size = 5
        dim2.stride = 5

        dimensions = []
        dimensions.append(dim1)
        dimensions.append(dim2)

        layout = MultiArrayLayout()
        layout.data_offset = 1
        layout.dim = dimensions

        newmsg = Int16MultiArray()
        newmsg.data = data
        newmsg.layout = layout

        self.obs_info_pub.publish(newmsg)


#######################################################################
#																      #
# 								MAIN	 						      #
#																      #
#######################################################################	

def main():
    rospy.init_node("[SIM] OBS node")
    node = SIM_ObstaclesNode()
    rospy.spin()

if __name__ == '__main__':
    main()
