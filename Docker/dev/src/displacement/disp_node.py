#!/usr/bin/env python3
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

"""
@file: disp_node.py
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

# import fonction du Pathfinder
from pathfinder.pathfinder import Pathfinder
from pathfinder.exceptions import PathNotFoundError, TimeOutError, DestBlockedError

# import msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int16, Float32MultiArray

# import utils
from disp_utils import *

# import comms
from disp_comm import init_comm
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

        log_info("Initializing Displacement Node.")

        ## Variables liées au match

        self.color = 0
        self.matchEnded = False

        ## Variables liées au fonctionnement de l'algorithme A* et de la création du chemin de points

        self.path = []
        self.pathfinder = Pathfinder(self.color)
        self.max_astar_time = MAX_ASTAR_TIME

        ## Variable liées au déplacement du robot
        self.move = False                       # Le robot est en cours de deplacement
        self.final_move = False                  # Le robot se dirige vers le point final
        self.turn = False
        self.final_turn = False 
        self.resume = False 
        self.paused = False                     # Le robot est arrete a cause d'un obstacle
        self.time_last_seen_obstacle = 0        # Temps auquel on a vu le dernier obstacle 
        self.refresh_update_obstacle = 0.5
        self.stop_detection_obstacle = False    # Desactivation de la surveillances des obstacles
        self.forward = True                     # Le robot est en marche avant ? (False = marche arriere)
        self.stop = False
        self.current_pos = [0,0,0]                # La position actuelle du robot

        # ## Variables de mode de DEPLACEMENT
        self.accurate = False             # Le robot se deplace precisement / lentement vers l'avant
        self.recalage = False               # Le robot se recale contre un mur / ou vient seulement en contact
        self.avoid_mode = False                  # Le robot se deplace en mode evitement
        self.rotation = False 

        # ## Variables speciales
        self.reset_point = [0,0]                 # Point au alentour duquel il faut reset les marges d'arret de l'evitement
        self.is_reset_possible = False            # Variable décrivant si il faut reset les marges d'evitement ou non
        self.is_first_accurate = False            # Variable permettant de savoir si le robot est dans un obstacle lors d'un evitement (savoir si on recule ou non)

        # ## Variables de gestion des obstacles / arrets
        self.blocked = False                     # Le robot est arrete a cause d'un obstacle

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
        self.pathfinder.set_max_astar_time(self.max_astar_time)

        # Instanciation de resultisInAvoidMode
        result = {'message':"", 'success':False, 'built path':[]}

        # On essaie d'obtenir un chemin
        try:
            result['built path'] = self.pathfinder.get_path(isInAvoidMode, isFirstAccurate, isSecondAttempt)
            result['message'] = "Path found" 
            result['success'] = True

        except PathNotFoundError:   
            result['message'] = "Path not found"
            result['success'] = False
            return result
        
        except TimeOutError:
            result['message'] = "Time out"
            result['success'] = False
            return result

        except DestBlockedError:
            result['message'] = "Dest Blocked"
            result['success'] = False
            return result

        # On recupere le chemin obtenu
        if not len(result['built path']):
            log_errs("Error - Empty path found")
            result['message'] = "Empty path found" 
            result['success'] = False
            return result
        
        self.path = result['built path']
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
        
        # S'il existe un point 
        # S'il existe un point intermediaire
        if len(self.path) >= 2:
            x = self.path[0][0]
            y = self.path[0][1]
            log_info("\nDisplacement Pass By ({}, {})".format(x,y))

            """ # TODO - remove patch
            x,y,_ = patch_frame_br(x,y,0,self.color) """

            # Si on est dans un obstacle
            if self.is_first_accurate:
                xLoc, _ = to_robot_coord(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.path[0])
                if xLoc > 0: # Marche avant necessaire
                    self.forward = True
                    pub_teensy.publish(Quaternion(x,y,self.current_pos[2],CMD_TEENSY["accurate"]))
                else:
                    self.forward = False
                    pub_teensy.publish(Quaternion(x,y,self.current_pos[2],CMD_TEENSY["accurate"]))
            else:
                self.forward = True
                pub_teensy.publish(Quaternion(x,y,0,CMD_TEENSY["disp"]))

            # Init params du mouvement
            self.turn = True
            self.final_move = False
            return

        # Sinon, s'il reste un point c'est le dernier
        if len(self.path) == 1:
            x, y, c = self.path[0]
            self.turn = True
            self.final_move = True

            """ # TODO - remove patch 
            x,y,c = patch_frame_br(x,y,c) """

            ## Gestion differents types de deplacement
            if self.accurate:
                xRob, yRob, cRob = self.current_pos

                log_info("\nDisplacement request ({}, {}, {}) accurate".format(x, y, c))
                
                xLoc, _ = to_robot_coord(xRob, yRob, cRob, [x,y,c])
                if xLoc >= 0:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["accurate"]))
                    self.forward = True
                else:
                    pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["accurate"]))
                    self.forward = False
                return

            if self.rotation:
                log_info("\nDisplacement request ({}, {}, {}) rotation.".format(x, y, c))
                #print("\n\n\n\nspeedRot = {}\n\n\n\n".format(self.speedRot))
                pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["rotation"]))
                return

            if self.recalage:
                log_info("\nDisplacement request ({}, {}, {}) recalage.".format(x, y, c))
                self.final_move = False     #Pas d'orientation finale en fin de recalage 
                pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["recalage"]))
                return
        
            ## Point final
            log_info("\nDisplacement request ({}, {}, {}) standard.".format(x, y, c))
            self.forward = True
            pub_teensy.publish(Quaternion(x, y, c, CMD_TEENSY["dispFinal"]))
            
        # Sinon, on on a fini (ou bien a nulle part ou aller, pcq obstacle
        # detecte sans qu'on bouge...)
        if len(self.path) == 0 and just_arrived:
            log_info("Arrived at destination!")
            # Reset des params
            self.move = False
            self.turn = False
            self.final_turn = False
            self.final_move = False

            self.avoid_mode = False
            self.recalage = False
            self.accurate = False
            self.rotation = False 

            # Publication a la strat
            pub_strat.publish(Int16(COM_STRAT["ok pos"]))


    def set_avoid_reset_point(self):
        """Calcul du point de reset des marges d'evitement.
        
        Il s'agit du point à partir duquel on sera suffissament loin de
        l'obstacle jusqu'a la fin du trajet."""

        avoid_robot_pos = self.pathfinder.get_robot_to_avoid_pos()[0]
        pos_list = [self.current_pos] + self.path
        i = len(pos_list) -1
        work = True
        while i > 0 and work: 
            #Parcours des droites de trajectoire de la fin vers le début
            if min(pos_list[i-1][0], pos_list[i][0])<avoid_robot_pos[0]<max(pos_list[i-1][0], pos_list[i][0]) or min(pos_list[i-1][1], pos_list[i][1])<avoid_robot_pos[1]<max(pos_list[i-1][1], pos_list[i][1]):
                if pos_list[i][0]-pos_list[i-1][0] !=0:
                    a = float(pos_list[i][1]-pos_list[i-1][1])/float(pos_list[i][0]-pos_list[i-1][0])
                    b=-1
                    c = pos_list[i-1][1]-a*pos_list[i-1][0]
                else:
                    a=1
                    b=0
                    c=-pos_list[i-1][0]
                #Calcul de la distance de la droite au centre de l'obstacle
                d = (a*avoid_robot_pos[0]+b*avoid_robot_pos[1]+c)/sqrt(a**2+b**2)
                if abs(d)<600:
                    work = False
            i-=1

        if work:    # On est toujours trop pres --> reset sur le point final
            self.reset_point = self.path[-1][:2]
        else:       # On calcul et setup le point de reset
            vect_normal = np.array([a,b])/np.linalg.norm([a,b])
            self.reset_point = np.array(avoid_robot_pos) - d*vect_normal
            
    def handle_obstacle(self, obstacle_seen, obstacle_stop, isDestBlocked):
        """Gestion d'arret du robot."""
        # dprint("Enter handle_obstacle()")
        if obstacle_stop:
            self.time_last_seen_obstacle = time()
            if not self.paused:
                log_info("New obstacle detected! - STOP")
                self.paused = True
                pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
                if isDestBlocked:
                    log_errs("Destination is being blocked.")
                    pub_strat.publish(Int16(COM_STRAT["stop blocked"]))
                else:
                    pub_strat.publish(Int16(COM_STRAT["stop"]))
            return

        if obstacle_seen:   # Il faut ralentir
            # dprint("Obstacle is seen")
            # self.time_last_seen_obstacle = time()

            log_info("New obstacle detected! - SPEED DOWN")
            self.paused = False
            pub_teensy.publish(Quaternion(0,0,0,CMD_TEENSY["stop"]))
            pub_strat.publish(Int16(COM_STRAT["go"]))
            return

        if self.paused and (time()-self.time_last_seen_obstacle > self.refresh_update_obstacle):
            self.paused = False
            self.resume = True
            log_info("Resume displacement.")
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
    
    # Wait for close key to quit
    rospy.spin()


#################################################################
if __name__ == '__main__':
    main()