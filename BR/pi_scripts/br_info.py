#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

from os import system

currentPosition = None
log = None
okVal = None

rospy.init_node("node_info")

def updateDisplay():
    system('clear')
    print(currentPosition)
    print("okPosition : " + str(okVal))
    if log != None:
        for n in range(len(log)):
            print(log[n])

def positionCallback(data):
    global currentPosition
    currentPosition = data
    updateDisplay()

def okCallback(data):
    global okVal
    okVal = data.data
    updateDisplay()
    
def logCallback(data):
    global log
    log = data.data
    updateDisplay()
    
position_subcriber = rospy.Subscriber("current_position", Pose2D, positionCallback)
ok_subcriber = rospy.Subscriber("okPosition", Int16, okCallback)
log_subcriber = rospy.Subscriber("logTotaleArray", Float32MultiArray, logCallback)

rospy.spin()

