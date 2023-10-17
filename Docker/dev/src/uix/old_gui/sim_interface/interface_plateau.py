#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pyright: reportMissingImports=false

from __future__ import division

import os
import roslib
import rospy

from geometry_msgs.msg import Pose2D, Quaternion
from std_msgs.msg import Int16, Int16MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray

from threading import RLock

import time
from math import cos, sin, atan2
from random import random, randint
import numpy as np
from ast import literal_eval

from mttkinter import mtTkinter as tk
import configparser as ConfigParser

from signal import signal, SIGINT
from sys import exit

import interface_utils
from interface_utils import X, Y, Xinput, Yinput, deg, dist

import interface_meta

def handler(signal_received, frame):
    # Handle any cleanup here
    rospy.loginfo("Received ctrl-c signal, closing interface ...")
    rospy.signal_shutdown(signal_received)
    exit(0)


dockerSimu = False
squareIndicators = True

class InterfaceNode:

    robot_name = "PR"  # par defaut
     
    RATIOV = 1

    SCALE = 1
    BACKGROUND_WIDTH = 1
    BACKGROUND_HEIGHT = 1

    XBORDERLEFT = 100  # marge à gauche de l'interface (mm)
    XBORDERRIGHT = 100  # marge à droite de l'interface (mm)
    YBORDER = 0  # marge en haut et en bas de l'interface (mm)

    DIM = 975/3000  # conversion mm en pixels (3m <-> 975px)

    HEIGHT = 975 + 2*YBORDER*DIM
    WIDTH = 650 + (XBORDERLEFT + XBORDERRIGHT)*DIM

    TAILLEOBST = 180

    PATHWIDTH = 8
    PATHCIRCLEWIDTH = 25
    PATHARROWLENGTH = 150

    SAMPLE_RADIUS = 75  # (150 de diamètre)
    EXCAVATION_SQUARE_SIDE = 150
    EXCAVATION_SQUARE_WIDTH = 22

    obstacles_Lidar = []
    obstacles_Sonars = []
    currentRobotX = 0
    currentRobotY = 0
    currentRobotTheta = 0

    isRobotPositionChanged = False

    isObstacleChanged = False
    isNewOrderReceived = False
    isNewPathReceived = False
    isGridDefined = False

    isExcavationSquareChanged = False

    msgPosition = 0

    color = 0  # par défaut

    squareLayout = [-1]*10  # couleur de chaque carré (-1:unknown | 0:home | 1:away)
    squareInfo = [0]*10  # état de chaque carré (0:up | 1:down)

    # fonction callback lors du renseignement de position: on recalcule l'emplacement de chaque coin du robot
    # un carré est ensuite tracé à l'aide de ces points (ainsi qu'un trait permettant de visualiser la direction)
    def update_position(self,msg):
        '''Se fait appeler par un subscriber si c'est avec la simulation 1 robot'''
        with self.lock:
            self.msgPosition = msg
            self.isRobotPositionChanged = True


    def update_position_multirobot(self,msg,index):
        '''Se fait appeler par le serveur TCP si c'est la simulation multirobot'''
        with self.lock:
            self.msgPosition[index] = msg
            self.isRobotPositionChanged = True


    def plot_Robot(self, msg, robot_name):
        with self.lock:

            if robot_name == "pr": ROBOTLENGTH, ROBOTDIAG, ROBOTANGLE = self.ROBOTLENGTH_pr, self.ROBOTDIAG_pr, self.ROBOTANGLE_pr
            elif robot_name == "gr": ROBOTLENGTH, ROBOTDIAG, ROBOTANGLE = self.ROBOTLENGTH_gr, self.ROBOTDIAG_gr, self.ROBOTANGLE_gr

            # TODO : en faire une erreur
            else: ROBOTLENGTH, ROBOTDIAG, ROBOTANGLE = self.ROBOTLENGTH_pr, self.ROBOTDIAG_pr, self.ROBOTANGLE_pr

            # Coordonnees des coins du robot
            coordinates = []
            for i in range (-1, 2, 2):
                for j in range (-1, 2, 2):
                    tmpx = msg.x + ROBOTDIAG*cos(msg.theta - i*np.pi/2 + j*ROBOTANGLE)
                    tmpy = msg.y + ROBOTDIAG*sin(msg.theta - i*np.pi/2 + j*ROBOTANGLE)
                    coordinates.append( (X(self, tmpx), Y(self, tmpy)) )


            x0 = X(self, msg.x)
            y0 = Y(self, msg.y)

            # Coordonnees du milieu du cote avant du robot
            x1 = X(self, msg.x + ROBOTLENGTH/2 * cos(msg.theta))
            y1 = Y(self, msg.y + ROBOTLENGTH/2 * sin(msg.theta))

            # Efface l'ancienne position du robot du canvas et remplace par la nouvelle
            self.canvas.delete("robot")
            self.canvas.create_polygon(coordinates, fill='red', tag = "robot")
            self.canvas.delete("direction")
            self.canvas.create_line(x0, y0, x1, y1, fill = 'pink', width = 5, tag = "direction")
            self.isRobotPositionChanged = False

        #on plot les obstacles (on regarde d'abord si c'est un obstacle détecté vi lidar ou sonar, on donne ensuite une couleur)
    def update_obstacles(self,msg):
        with self.lock:
            self.msgObstacle = msg
            self.isObstacleChanged = True

    def plot_obstacles(self,msg):
        with self.lock:
            newObstacles = []
            for i in range((msg.layout.dim[0]).size):
                newObstacles.append([msg.data[5*i + j +1] for j in range(5)])
            if msg.data[0]:
                color = "blue"
                tag1 = "sonars"
                tag2 = "vsonars"
            else :
                color = "red"
                tag1 = "lidar"
                tag2 = "vlidar"

            self.canvas.delete(tag1)
            self.canvas.delete(tag2)

            for obstacle in newObstacles:
                x0 = X(self, obstacle[0])
                y0 = Y(self, obstacle[1])
                x1 = X(self, obstacle[0] - self.TAILLEOBST/2)
                y1 = Y(self, obstacle[1] + self.TAILLEOBST/2)
                x2 = X(self, obstacle[0] + self.TAILLEOBST/2)
                y2 = Y(self, obstacle[1] - self.TAILLEOBST/2)
                points = [x1, y1, x2, y2]
                x3 = X(self, obstacle[0] + self.RATIOV * obstacle[3])
                y3 = Y(self, obstacle[1] + self.RATIOV * obstacle[4])
                vpoints = [x0, y0, x3, y3]
                self.canvas.create_oval(points, fill = color, tag = tag1)
                self.canvas.create_line(vpoints, fill = 'green', tag = tag2)
            self.isObstacleChanged = False

    def noteClickPosition(self, evt):
        x = Xinput(self, evt.x)
        y = Yinput(self, evt.y)
        self.clickPosition = [x, y]
        self.canvas.create_oval(x-10,y-10,x+10,y+10,fill = 'yellow' ,tag = "clickPosition")
        
    def sendOrder(self, evt):
        x = Xinput(self, evt.x)
        y = Yinput(self, evt.y)
        self.canvas.delete("clickPosition")
        rospy.loginfo("click received at " + str(x) + "," + str(y))
        self.pubOrder.publish(Quaternion(self.clickPosition[0],self.clickPosition[1] , atan2(y-self.clickPosition[1],x-self.clickPosition[0]), 0))

    def update_Order(self, msg):
        with self.lock:
            if msg.w in [0,1,5,6,7,8,10,11]:
                self.msgOrder = msg
                self.isNewOrderReceived = True

    def plot_Order(self, msgOrder):
        with self.lock:
            self.canvas.delete("order")
            self.canvas.create_oval(X(self, msgOrder.x)-10, Y(self, msgOrder.y)-10, X(self, msgOrder.x)+10, Y(self, msgOrder.y)+10,fill = 'purple' ,tag = "order")
            self.isNewOrderReceived = False


    
    

    create_path_circle = interface_utils.create_path_circle
    create_path_line = interface_utils.create_path_line
    create_path_arrow = interface_utils.create_path_arrow

    update_path = interface_meta.update_path
    plot_path = interface_meta.plot_path

    create_grid_circle = interface_meta.create_grid_circle

    update_grid = interface_meta.update_grid
    plot_grid = interface_meta.plot_grid


    # create_bouee_number = interface_utils.create_bouee_number
    # update_bouees = interface_utils.update_bouees


    create_sample = interface_utils.create_sample
    update_sample = interface_utils.update_samples

    create_excavation_square = interface_utils.create_excavation_square
    plot_excavation_squares = interface_utils.plot_excavation_squares

    create_square_indicator = interface_utils.create_square_indicator
    plot_square_indicators = interface_utils.plot_square_indicators


    def update_color(self, msg):

        self.color = msg.data

        self.canvas.delete("path_circle")
        self.canvas.delete("path_line")

        self.canvas.delete('lidar')
        self.canvas.delete('vlidar')






    def update_squareLayout(self, msg):
        with self.lock:

            # Reset des carrés de fouille
            self.squareInfo = [0]*10
            self.squareLayout = self.pc[msg.data]

            self.isExcavationSquareChanged = True
            

    def update_squareInfo(self, msg):
         with self.lock:
            print(self.squareInfo, self.squareLayout)

            self.squareInfo[msg.data // 10] = msg.data % 10
            self.isExcavationSquareChanged = True


    def refresh(self):
        with self.lock:  # verrouille l'acces memoire pendant l'affichage

            # TODO : A NE LAISSER QU'EN CAS DE SIMULATION 1 ROBOT
            #if not dockerSimu: self.robot_name = rospy.get_param("robot_name")  # on update le nom du robot qu'on est en train de simuler
            self.robot_name = "pr"  # TODO : à paramétrer

            if self.isNewPathReceived:
                self.plot_path(self.msgPath, self.msgPosition)

            if self.isGridDefined:
                self.plot_grid(self.nodeGrid)

            if self.isObstacleChanged:
                self.plot_obstacles(self.msgObstacle)

            if self.isRobotPositionChanged:

                if not dockerSimu:
                    self.plot_Robot(self.msgPosition, self.robot_name)  # msgPosition peut contenir une pos ou pluseurs pos en fonction du type de simulation
                else:
                    self.plot_Robot(self.msgPosition, "pr")
                    self.plot_Robot(self.msgPosition, "gr")

            if self.isExcavationSquareChanged:
                self.plot_excavation_squares()
                if squareIndicators: self.plot_square_indicators()

            if self.isNewOrderReceived:
                self.plot_Order(self.msgOrder)
            self.fenetre.after(10,self.refresh)



    def __init__(self):

        self.lock = RLock()  # on crée un verrou qui evite de modifier les valeurs en memoire alors qu'on est en train de les afficher
        
        rospy.init_node('InterfaceNode')
        rospy.loginfo("Initialisation de l'interface")

        #rospy.set_param("robot_name", "titanic")  # par defaut

        self.reader_pr = ConfigParser.ConfigParser(inline_comment_prefixes = ";")
        self.reader_gr = ConfigParser.ConfigParser(inline_comment_prefixes = ";")


        # Initialisation des variables pr

        self.reader_pr.read(os.path.join(os.path.dirname(__file__),"../pr_start.ini"))

        self.ROBOTWIDTH_pr = int(self.reader_pr.get("Robot", "robot_larg"))
        self.ROBOTLENGTH_pr = int(self.reader_pr.get("Robot", "robot_long"))
        self.ROBOTDIAG_pr = np.linalg.norm([self.ROBOTWIDTH_pr/2, self.ROBOTLENGTH_pr/2])
        self.ROBOTANGLE_pr = np.arctan(self.ROBOTLENGTH_pr/self.ROBOTWIDTH_pr)


        # Initialisation des variables gr
        self.reader_gr.read(os.path.join(os.path.dirname(__file__),"../gr_start.ini"))
        
        self.ROBOTWIDTH_gr = int(self.reader_gr.get("Robot", "robot_larg"))
        self.ROBOTLENGTH_gr = int(self.reader_gr.get("Robot", "robot_long"))
        self.ROBOTDIAG_gr = np.linalg.norm([self.ROBOTWIDTH_gr/2, self.ROBOTLENGTH_gr/2])
        self.ROBOTANGLE_gr = np.arctan(self.ROBOTLENGTH_gr/self.ROBOTWIDTH_gr)



        #initialisation de tkinter et du canvas
        self.fenetre = tk.Tk()
        self.fenetre.title('InterfaceNodePlateauOnly')
        self.canvas = tk.Canvas(self.fenetre, borderwidth = 0, width = self.WIDTH, height = self.HEIGHT)
        self.canvas.pack(expand=True)
        self.photo = tk.PhotoImage(file=os.path.join(os.path.dirname(__file__),"Background_Interface.gif"))
        self.canvas.create_image(self.XBORDERLEFT*self.DIM, self.YBORDER*self.DIM, anchor='nw', image=self.photo)

        #self.commande = tk.StringVar(value ="commande")
        #self.etatwindow = Message(self.canvas.create_window(802,0, anchor = 'nw', height = 600, width = 400), textvariable = self.etat )

        self.canvas.pack()
        
        self.canvas.bind('<Button-3>',self.noteClickPosition)
        self.canvas.bind('<ButtonRelease-3>',self.sendOrder)


        # initialisation des suscribers
        self.sub_start=rospy.Subscriber("/color", Int16, self.update_color)

        self.sub_square_layout=rospy.Subscriber("/simu/squareLayout", Int16, self.update_squareLayout)  # TODO : à ne laisser que lors des simulations
        self.sub_square_info=rospy.Subscriber("/simu/squareInfo", Int16, self.update_squareInfo)  # TODO : à ne laisser que lors des simulations

        self.sub_path = rospy.Subscriber("/simu/current_path", Float32MultiArray, self.update_path)
        self.sub_grid = rospy.Subscriber("/simu/nodegrid", Float32MultiArray, self.update_grid)

        # TODO : A NE LAISSER QU'EN CAS DE SIMULATION 1 ROBOT
        if not dockerSimu: self.sub_pos=rospy.Subscriber("/current_position", Pose2D, self.update_position)


        self.sub_obstacles=rospy.Subscriber("/obstaclesInfo", Int16MultiArray, self.update_obstacles)
        self.sub_Order=rospy.Subscriber("/nextPositionTeensy",Quaternion,self.update_Order)

        #initialisation des publishers
        self.pubOrder = rospy.Publisher("/nextPositionTeensy", Quaternion, queue_size=10, latch=False)         #### A CHANGER POUR ACTIVER L'ACTION RDVPOS POUR PILOTER LE ROBOT DEPUIS L'INTERFACE ####
        rospy.loginfo("Fin d'initialisation de l'interface")




        '''
        #### Création des object de départ
        sampleColors = ("blue", "green", "red")

        # TODO : à mettre dans une loop en important les coords des samples
        self.create_sample(555, 900, 0, self.SAMPLE_RADIUS, 0, "blue")
        self.create_sample(675, 830, 0, self.SAMPLE_RADIUS, 1, "green")
        self.create_sample(795, 900, 0, self.SAMPLE_RADIUS, 2, "red")

        # Génération des positions aléatoires des échantillons du site de fouille
        # coords du site : (1200, 800, 1550, 1150)
        squareSamplesPos = []
        for k in range(3): squareSamplesPos.append((randint(1200 + self.SAMPLE_RADIUS, 1550 - self.SAMPLE_RADIUS), randint(800 + self.SAMPLE_RADIUS, 1150 - self.SAMPLE_RADIUS)))

        squareSamplesTries = 0
        while (dist(squareSamplesPos[0], squareSamplesPos[1]) < 2*self.SAMPLE_RADIUS 
               or dist(squareSamplesPos[1], squareSamplesPos[2]) < 2*self.SAMPLE_RADIUS
               or dist(squareSamplesPos[0], squareSamplesPos[2]) < 2*self.SAMPLE_RADIUS):
            for k in range(3): squareSamplesPos[k] = (randint(1200 + self.SAMPLE_RADIUS, 1550 - self.SAMPLE_RADIUS), randint(800 + self.SAMPLE_RADIUS, 1150 - self.SAMPLE_RADIUS))
            squareSamplesTries += 1
        print("Nombre d'essais pour trouver une config du site de fouille : {}".format(squareSamplesTries))

        for k in range(3):
            self.create_sample(squareSamplesPos[k][0], squareSamplesPos[k][1], (random()-0.5)*2*np.pi, self.SAMPLE_RADIUS, k + 3, sampleColors[k])
        
        # Creating position for all excavation squares and selecting one of the possible configurations

        # TODO : mettre en constantes
        # 0 : home side | 1 : away side | 2 : red cross
        pc_1 = [0, 0, 2, 1, 0, 0, 1, 2, 1, 1]
        pc_2 = [2, 0, 0, 0, 1, 1, 0, 1, 1, 2]
        pc_3 = [0, 0, 2, 0, 1, 1, 0, 2, 1, 1]
        pc_4 = [2, 0, 0, 1, 0, 0, 1, 1, 1, 2]
        self.pc = (pc_1, pc_2, pc_3, pc_4)

        # TODO : mettre en constantes
        self.excavationSquarePos_x = 2000 + 11
        self.excavationSquarePos_y = [667.5, 852.5, 1037.5, 1222.5, 1407.5, 1592.5, 1777.5, 1962.5, 2147.5, 2332.5]

        self.plot_excavation_squares()

        if squareIndicators: self.plot_square_indicators()
        '''

        self.fenetre.after(10,self.refresh)  # every 10ms we refresh


if __name__ == '__main__':
    node = InterfaceNode()
    signal(SIGINT, handler)
    node.fenetre.mainloop()
 
