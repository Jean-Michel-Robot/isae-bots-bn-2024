#! /usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion

from numpy import mean

log = None

rospy.init_node("node_odos")
position_pub = rospy.Publisher("nextPositionTeensy", Quaternion, queue_size=10)


def sendOrder(x,y,z,w):
    msg = Quaternion()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    msg.w = float(w)
    position_pub.publish(msg)

def logCallback(data):
    global log
    log = data.data

log_subcriber = rospy.Subscriber("odos_count", Float32MultiArray, logCallback)

def odosLigneDroite():
    
    global log

    fileLigneDroite = open("logLigneDroite.log", "a")

    LR = []
    Units = [] 

    continueInput = 'y'
    keepInput = 'y'

    while continueInput != 'n':

        raw_input("Press enter to start ")
        Li = log[0]
        Ri = log[1]
        
        raw_input("Press enter to stop ")
        Lf = log[0]
        Rf = log[1]

        dL = Lf - Li
        dR = Rf - Ri + 1

        d = input("Enter distance (mm) : ")

        keepInput = raw_input("Keep this try ? [y/n] ")
        if(keepInput != 'n'):
            LR.append(dL/dR)
            Units.append(dL/d)
            fileLigneDroite.write(str((d,Li,Ri,Lf,Rf)) + "\n")

        print("LR : " + str(LR) + ", avg = " + str(mean(LR)))
        print("Units : " + str(Units) + ", avg = " + str(mean(Units)))
        
        continueInput = raw_input("Continue ? [y/n]")

    return(mean(LR), mean(Units))

def odosRotation(LR = 1.):

    global log

    fileRotation = open("logRotation.log", "a")

    Ecarts = []

    continueInput = 'y'
    keepInput = 'y'

    while continueInput != 'n':

        raw_input("Press enter to start ")
        Li = log[0]
        Ri = log[1]

        rotSpeed = input("Rotation speed ? ")

        raw_input("Press enter to rotate ")
        sendOrder(rotSpeed,-rotSpeed,0,4)
        
        raw_input("Press enter to stop the rotation ")
        sendOrder(0,0,0,4)

        raw_input("Press enter to stop the measure ")
        Lf = log[0]
        Rf = log[1]

        dL = Lf - Li
        dR = (Rf - Ri)*LR + 1

        d = input("Enter distance (rad) : ")

        keepInput = raw_input("Keep this try ? [y/n] ")
        if(keepInput != 'n'):
            Ecarts.append((dL-dR)/d)
            fileRotation.write(str((d,Li,Ri,Lf,Rf)) + "\n")

        print("Ecarts : " + str(Ecarts) + ", avg = " + str(mean(Ecarts)))
        
        continueInput = raw_input("Continue ? [y/n] ")
        
    return(mean(Ecarts))


(LR,E) = odosLigneDroite()
Ecarts = odosRotation(LR)
print(LR)
print(E)
print(Ecarts)



        
        


        








    
