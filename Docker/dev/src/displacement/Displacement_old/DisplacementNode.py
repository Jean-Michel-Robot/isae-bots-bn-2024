#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
@file: DisplacementNode.py
@status: OK

Fichier implementant le DisplacementNode.

Ce node intervient dans la gestion des deplacements sur commandes du
ActionNode de la strat. Il fait le lien entre la strat et la teensy.
Fichier executable du dossier.

NB: code [english], commentaires [french].
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################
import sys
import rospy
import numpy as np
from math import sqrt
from time import time

print("version : ",sys.version)

# import fonction du Pathfinder
from pathfinder.pathfinder import Pathfinder
from pathfinder.exceptions import PathNotFoundError, TimeOutError

# import msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16

# import init functions
from disp_gain import init_gain
from disp_comm import init_comm

# import utils
from disp_utils import LOG_INFO, LOG_ERRS, dprint
from disp_utils import toRobotCoord, patchFrameBR

# import comms
from disp_comm import COM_STRAT, CMD_TEENSY
from disp_comm import pub_strat, pub_teensy

#################################################################
#																#
# 						DISPLACEMENT NODE						#
#																#
#################################################################

class DisplacementNode:

    """Classe du implementant le DisplacementNode."""

#######################################################################
# Initialisation 
#######################################################################
 
    def __init__(self):
        """Initialise le DisplacementNode."""

        LOG_INFO("Initializing Displacement Node.")
       
        ## Variable d'etat d'un DEPLACEMENT
        self.turn = False                       # Le robot tourne sur lui meme
        self.move = False                       # Le robot est en cours de deplacement
        self.finalTurn = False                  # Le robot effectue sa rotation finale
        self.finalMove = False                  # Le robot se dirige vers le point final
        self.forward = True                     # Le robot est en marche avant ? (False = marche arriere)

        ## Variables de mode de DEPLCEMENT
        self.accurateMode = False               # Le robot se deplace precisement / lentement 
        self.rotationMode = False               # Le robot tourne seulement sur lui meme 
        self.recalageMode = False               # Le robot se recale contre un mur / ou vient seulement en contact
        self.arAccurateMode = False             # Le robot est en mode accurate en marche arrière force
        self.avAccurateMode = False             # Le robot est en mode accurate en marche avant force
        self.sameXMode = False                  # Le robot se deplace au point donne en gardant son X actuel
        self.avoidMode = False                  # Le robot se deplace en mode evitement

        ## Variables speciales
        self.finish = False                     # Le match est fini, on 'bloque' le robot
        self.resetPoint = [0,0]                 # Point au alentour duquel il faut reset les marges d'arret de l'evitement
        self.isResetPossible = False            # Variable décrivant si il faut reset les marges d'evitement ou non
        self.isFirstAccurate = [False]          # Variable permettant de savoir si le robot est dans un obstacle lors d'un evitement (savoir si on recule ou non)
        self.recalageParam = CMD_TEENSY["stop"] # Parametre de recalage (avant/arrière ; contact ?)

        ## Variables de gestion des obstacles / arrets
        self.resume = False                     # On repart apres avoir rencontre un obstacle
        self.paused = False                     # Le robot est arrete a cause d'un obstacle
        self.time_last_seen_obstacle = 0        # Temps auquel on a vu le dernier obstacle 
        self.refresh_update_obstacle = 0.5
        self.stop_detection_obstacle = False    # Desactivation de la surveillances des obstacles

        ## Variables de jeu
        self.color_int = 0                      # HOME by default
        self.color_txt = "Yellow"               # HOME = yellow this year

        self.current_pos = None

        self.path = []
        self.pathfinder = Pathfinder(self.color_int)

        self.maxAstarTime = 5

#######################################################################
# Fonctions de construction de path
#######################################################################

    def build_path(self, isInAvoidMode, isFirstAccurate, isSecondAttempt):
        """Fonction appelee quand on cherche chemin et le pathfinder setup.
        
        Retourne un chemin de la forme:
            [[x_1, y_1, theta_1], ..., [x_dest, y_dest, theta_dest]]

        avec (x_1, y_1), ... (x_n, y_n) fournis par le pathfinder, et les 
        angles calcules tels que:
            
        Y  
        ^
        |      (x2, y2)
        |    +
        |
        |    ^
        |    | theta1
        |    +
        |      (x1,y1)
        |  ^
        | /
        |/ theta0    
        +----------> X
        (x0,y0)

        le resultat de la fonction est un dictionnaire qui contient:
            - msg: un message d'erreur le cas echeant
            - success: si un chemin a ete trouve
            - chemin_calcule: le chemin calcule."""

        # Update temps max de l'Astar
        self.pathfinder.setMaxAstarTime(self.maxAstarTime)

        # Instanciation de result
        result = {'message':"", 'success':False, 'built path':[]}

        # On essaie d'obtenir un chemin
        try:
            result['built path'] = self.pathfinder.getPath(isInAvoidMode, isFirstAccurate, isSecondAttempt)
            result['message'] = "PathFound"
            result['success'] = True

        except PathNotFoundError:
            result['message'] = "Path not found"
            result['success'] = False
            return result
        
        except TimeOutError:
            result['message'] = "Time out"
            result['success'] = False
            return result

        # On recupere le chemin obtenu
        built_path = result['built path']
        if not len(built_path):
            LOG_ERRS("Error - not len(built_path) in build_path().")
            result['success'] = False
            return result
        
        self.path = built_path
        return result


    def next_point(self, just_arrived):
        """Envoie une commande du prochain point a la Teensy.
        
        - Soit on vient d'arriver au point (just_arrived=True) --> on supp 
        le premier point du path.
        - Soit on etait a un obstacle ou on vient de repartir --> on garde 
        le premier point du path (dest temporaire).
        
        Si il n'y a plus de point, on vient d'arriver, notif a la strat.
        Et gestion des differents modes de deplacements si param avant."""

        # Si on vient d'arriver, on retire le premier point si possible
        if just_arrived:
            if len(self.path) > 0:
                self.path = self.path[1:]
        
        # S'il existe un point intermediaire
        if len(self.path) >= 2:
            x = self.path[0][0]
            y = self.path[0][1]
            LOG_INFO("\nDisplacement Pass By ({}, {})".format(x,y))

            # TODO - remove patch
            x,y,_ = patchFrameBR(x,y,0)

            # Si on est dans un obstacle
            if self.isFirstAccurate[0]:
                xLoc, _ = toRobotCoord(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.path[0])
                if xLoc > 0: # Marche avant necessaire
                    self.forward = True
                    pub_teensy.publish(Quaternion(x,y,self.current_pos[2],CMD_TEENSY["light final av"]))
                else:
                    self.forward = False
                    pub_teensy.publish(Quaternion(x,y,self.current_pos[2],CMD_TEENSY["light final ar"]))
            else:
                self.forward = True
                pub_teensy.publish(Quaternion(x,y,0,CMD_TEENSY["pass_by"]))

            # Init params du mouvement
            self.turn = True
            self.finalMove = False
            return

        # Sinon, s'il reste un point c'est le dernier
        if len(self.path) == 1:
            x, y, c = self.path[0]
            self.turn = True
            self.finalMove = True

            # TODO - remove patch 
            x,y,c = patchFrameBR(x,y,c)

            ## Gestion differents types de deplacement
            if self.accurateMode:
                xRob, yRob, cRob = self.current_pos
                if self.sameXMode: x = xRob

                LOG_INFO("\nDisplacement request ({}, {}, {}) accurate".format(x, y, c))

                if self.arAccurateMode:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["light final ar"]))
                    self.forward = False
                    return
                if self.avAccurateMode:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["light final av"]))
                    self.forward = True
                    return
                
                xLoc, _ = toRobotCoord(xRob, yRob, cRob, [x,y,c])
                if xLoc >= 0:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["light final av"]))
                    self.forward = True
                else:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["light final ar"]))
                    self.forward = False
                return

            if self.rotationMode:
                LOG_INFO("\nDisplacement request ({}, {}, {}) rotation.".format(x, y, c))
                #print("\n\n\n\nspeedRot = {}\n\n\n\n".format(self.speedRot))
                pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["rotation"]))
                return

            if self.recalageMode:
                LOG_INFO("\nDisplacement request ({}, {}, {}) recalage.".format(x, y, c))
                self.finalMove = False     #Pas d'orientation finale en fin de recalage 
                pub_teensy.publish(Quaternion(x, y, c, self.recalageParam))
                return
        
            ## Point final
            LOG_INFO("\nDisplacement request ({}, {}, {}) standard.".format(x, y, c))
            self.forward = True
            pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["go_to"]))
            
        # Sinon, on on a fini (ou bien a nulle part ou aller, pcq obstacle
        # detecte sans qu'on bouge...)
        if len(self.path) == 0 and just_arrived:
            LOG_INFO("Arrived at destination!")
            # Reset des params
            self.move = False
            self.turn = False
            self.finalTurn = False
            self.finalMove = False

            self.avoidMode = False
            self.sameXMode = False
            self.recalageMode = False
            self.accurateMode = False
            self.rotationMode = False 
            self.arAccurateMode = False     
            self.avAccurateMode = False

            # Publication a la strat
            pub_strat.publish(Int16(COM_STRAT["ok pos"]))


    def setAvoidResetPoint(self):
        """Calcul du point de reset des marges d'evitement.
        
        Il s'agit du point à partir duquel on sera suffissament loin de
        l'obstacle jusqu'a la fin du trajet."""

        avoidRobotPos = self.pathfinder.getRobotToAvoidPos()[0]
        posList = [self.current_pos] + self.path
        i = len(posList) -1
        work = True
        while i > 0 and work: 
            #Parcours des droites de trajectoire de la fin vers le début
            if min(posList[i-1][0], posList[i][0])<avoidRobotPos[0]<max(posList[i-1][0], posList[i][0]) or min(posList[i-1][1], posList[i][1])<avoidRobotPos[1]<max(posList[i-1][1], posList[i][1]):
                if posList[i][0]-posList[i-1][0] !=0:
                    a = float(posList[i][1]-posList[i-1][1])/float(posList[i][0]-posList[i-1][0])
                    b=-1
                    c = posList[i-1][1]-a*posList[i-1][0]
                else:
                    a=1
                    b=0
                    c=-posList[i-1][0]
                #Calcul de la distance de la droite au centre de l'obstacle
                d = (a*avoidRobotPos[0]+b*avoidRobotPos[1]+c)/sqrt(a**2+b**2)
                if abs(d)<600:
                    work = False
            i-=1

        if work:    # On est toujours trop pres --> reset sur le point final
            self.resetPoint = self.path[-1][:2]
        else:       # On calcul et setup le point de reset
            vectN = np.array([a,b])/np.linalg.norm([a,b])
            self.resetPoint = np.array(avoidRobotPos) - d*vectN
            
    def handle_obstacle(self, obstacle_seen, obstacle_stop, isDestBlocked):
        """Gestion d'arret du robot."""
        # dprint("Enter handle_obstacle()")
        if obstacle_stop:
            self.time_last_seen_obstacle = time()
            if not self.paused:
                LOG_INFO("New obstacle detected! - STOP")
                self.paused = True
                pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
                if isDestBlocked:
                    LOG_ERRS("Destination is being blocked.")
                    pub_strat.publish(Int16(COM_STRAT["stop blocked"]))
                else:
                    pub_strat.publish(Int16(COM_STRAT["stop"]))
            return

        if obstacle_seen:   # Il faut ralentir
            # dprint("Obstacle is seen")
            # self.time_last_seen_obstacle = time()

            LOG_INFO("New obstacle detected! - SPEED DOWN")
            self.paused = False
            pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
            pub_strat.publish(Int16(COM_STRAT["go"]))
            return

        if self.paused and (time()-self.time_last_seen_obstacle > self.refresh_update_obstacle):
            self.paused = False
            self.resume = True
            LOG_INFO("Resume displacement.")
            pub_strat.publish(Int16(COM_STRAT["go"]))
            self.next_point(False)



#################################################################
#																#
# 							MAIN PROG 							#
#																#
#################################################################

def main():
    # Init and create DecisionNode
    rospy.init_node("DisplacementNode")
    node = DisplacementNode()

    init_comm(node)
    init_gain(node)
    
    # Wait for close key to quit
    rospy.spin()


#################################################################
if __name__ == '__main__':
    main()