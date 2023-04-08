#! /usr/bin/python
# -*- coding: utf-8 -*-


from consolemenu import *

from consolemenu.items import *

import rospy 
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D

from time import sleep



rospy.init_node("node_action")
position_pub = rospy.Publisher("nextPositionTeensy", Quaternion, queue_size=10)


def sendOrder(x,y,z,w):
    msg = Quaternion()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    msg.w = float(w)
    position_pub.publish(msg)
    sleep(1)



def askPosition():
    x = input("x ? ")
    y = input("y ? ")
    tetha = input("tetha ? ")
    return(x,y,tetha)

def goToPosition(askBool,x, y, tetha):
    if(askBool):
        (x,y,tetha) = askPosition()
    sendOrder(x,y,tetha,0)

def updatePosition(askBool,x, y, tetha):
    if(askBool):
        (x,y,tetha) = askPosition()
    sendOrder(x,y,tetha,3)

def askForward():
    d = input("Distance (in millimeters) ? ")
    updatePosition(False, 0,0,0)
    goToPosition(False, d, 0,0)

def orientation(askBool, tetha):    
    if(askBool):
        tetha = input("tetha ? ")
    else:
       updatePosition(False,0,0,0)     
    sendOrder(0,0,tetha,9)


#Main menu
menu = ConsoleMenu("Main menu")

#Moving bot
moving_sub = ConsoleMenu("Moving")

moving_items = []
moving_items.append(FunctionItem("Go forward (resets positions)", askForward))
moving_items.append(FunctionItem("Go to (0,0,0)", goToPosition,  [False,0.,0.,0.]))
moving_items.append(FunctionItem("Go to...", goToPosition, [True,0.,0.,0.]))
moving_items.append(FunctionItem("Do a 180 turn (resets postition)" , orientation, [False,3.1415]))
moving_items.append(FunctionItem("Do an orientation of..." , orientation, [True, 0]))
moving_items.append(FunctionItem("Reset position to (0,0,0)", updatePosition, [False,0,0,0]))
moving_items.append(FunctionItem("Set position to...", updatePosition, [True,0,0,0]))

for item in moving_items :
    moving_sub.append_item(item)

#Direct_command
direct_sub = ConsoleMenu("Direct control")

direct_items = []
direct_items.append(FunctionItem("Go forward (at 80%)", sendOrder, [80,80,0,4]))
direct_items.append(FunctionItem("Go backward (at 80%)", sendOrder, [-80,-80,0,4]))
direct_items.append(FunctionItem("Rotation clockwise (at 80%)", sendOrder, [80,-80,0,4]))
direct_items.append(FunctionItem("Rotation clockwise (at 80%)", sendOrder, [-80,80,0,4]))
direct_items.append(FunctionItem("Stop the bot", sendOrder, [0,0,0,4]))

for item in direct_items :
    direct_sub.append_item(item)


moving_item = SubmenuItem("Move the bot", moving_sub)
direct_cmd_item = SubmenuItem("Directly control the motors", direct_sub)

menu.append_item(moving_item)
menu.append_item(direct_cmd_item)

menu.show()

