#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D

from os import system
import sys
from time import time

currentPosition = None
log = None
okVal = None
lastUpdateTime = time() 

rospy.init_node("node_info")

def updateDisplay():
    global lastUpdateTime
    if time() > (lastUpdateTime + 0.10):
        string = str(currentPosition) + "\nokPosition : " + str(okVal)
        for i in log:
            string +="\n" + str(i)
        system('clear')
        print(string)
        lastUpdateTime = time()



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

