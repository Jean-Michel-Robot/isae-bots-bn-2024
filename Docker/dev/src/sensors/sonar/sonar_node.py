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


#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os
import rospy
import numpy as np
import configparser
import ast
from sonar_lib import coord_obstacle
from std_msgs.msg      import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import Pose2D, Point

#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[SON]"
# _BOT_NAME_ = rospy.get_param("robot_name")
_CFG_FILE_ = "gr_config.ini"

def log_info(msg):
    """
    Print logs standard.
    """
    rospy.loginfo(f"{_NODENAME_} {msg}")

#################################################################
#                                                               #
#                         SONAR node                            #
#                                                               #
#################################################################

class SonarNode:
    """
    ROS node SONAR node for sonar obstacles detection.
    """

    def __init__(self):
        log_info("Initializing SonarNode ...")
        
        # Get sonars from config file of the robot
        reader = configparser.ConfigParser()
        reader.read(os.path.join(os.path.dirname(__file__),f"../../../{_CFG_FILE_}"))
        
        self.nb_sonars = 0
        self.sonars_lst = []     #on enregistre chaque sonar (sa position et son cap) dans cette liste
        self.sonars_pos = [[0 for i in range(3)] for j in range(4)]
        
        for k, v in reader.items("SONAR"):
            if k[:5] != 'sonar': continue
            self.sonars_lst.append(list(ast.literal_eval(v)))
            self.nb_sonars += 1

        self.obs_pub = rospy.Publisher("/sensors/obstaclesSonar", Int16MultiArray, queue_size=10, latch=False)
        self.pos_sub = rospy.Subscriber("/current_position", Pose2D, self.recv_position)
        self.son_sub = rospy.Subscriber("/ultrasonicDistances", Point, self.recv_obstacle)  # can change topic name ?
    
    def recv_position(self, msg):
        """
        Feedback on current position /disp/current_position topic.
        """
        for i in range(self.nb_sonars) :
            [x, y, dir_visee] = self.sonars_lst[i]
            pol_dst = np.hypot(x, y)
            pol_cap = np.arctan2(y, x)
            dir_visee = dir_visee * np.pi/180
            self.sonars_pos[i][0] = msg.x + pol_dst * np.cos(msg.theta + pol_cap)
            self.sonars_pos[i][1] = msg.y + pol_dst * np.sin(msg.theta + pol_cap)
            self.sonars_pos[i][2] = msg.theta + dir_visee

        self.x_robot = msg.x
        self.y_robot = msg.y
        self.cap = msg.theta

    def recv_obstacle(self, msg):
        """
        Feedback on obstacle info from sonars
        """
        obs_lst = []
        dst_lst = [msg.x, msg.y]

        for i in range(self.nb_sonars):
            dst = dst_lst[i]
            pos = self.sonars_pos[i]
            coord = coord_obstacle(pos, dst)
            if coord : obs_lst.append(coord)

        info = Int16MultiArray()
        data = [1]
        for pos in obs_lst:
            for coord in pos:
                data.append((int)(coord))
        info.data = data

        layout = MultiArrayLayout()
        layout.data_offset = 1

        dims = []
        dim1 = MultiArrayDimension()
        dim1.label = "nbObstacles"
        dim1.size = len(obs_lst)
        dim1.stride = 2* len(obs_lst)

        dim2 = MultiArrayDimension()
        dim2.label = "coordinates"
        dim2.size = 2
        dim2.stride = 2

        dims.append(dim1)
        dims.append(dim2)
        layout.dim = dims
        info.layout = layout

        self.obs_pub.publish(info)


#################################################################
#                                                               #
#                             MAIN                              #
#                                                               #
#################################################################

def main():
    rospy.init_node("SonarNode")
    node = SonarNode()
    rospy.spin()


if __name__ == '__main__':
    main()
