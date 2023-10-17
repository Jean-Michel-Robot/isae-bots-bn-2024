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
@file: astar.py
@status: in progress

Librairie implementant un algorithme de A* sur un ensemble de noeud et 
d'obstacle définie dans la classe Map.
"""

#######################################################################
#
#                               IMPORTS
#
#######################################################################

import time


from pathfinder.node import Node
from queue import PriorityQueue
from pathfinder.exceptions import PathNotFoundError, TimeOutError, DestBlockedError

### CONSTANTES ########################################################
X_THRESHOLD = 5     # 5mm
Y_THRESHOLD = 5     # 5mm
#######################################################################

def is_node_in_list(list, node):
    #Renvoie True si le noeud demandé est dans la liste et False sinon
    for test_node in list:
        if test_node.equals(node):
            return True, test_node
    return False

def is_node_out_obstacles(tableMap, node, isFirst):
    #Teste si le noeud est en dehors des obstacles
    if not tableMap.get_avoid():
        return True
    if not isFirst:
        return True
    for obstacle in tableMap.get_obstacle_list():
        if obstacle.is_node_in(node):
            """ print(obstacle.get_name())
            print(obstacle.get_x_center())
            print(obstacle.get_radius()) """
            return False
    return True


#######################################################################
#
#                           Algo A star
#
#######################################################################
    
def a_star(init, goal, tableMap, isFirstAccurate, maxAstarTime):
    """Algorithme du A* renvoyant un chemin entre start et goal"""

#######################################################################
# PHASE D'INITIALISATION
#######################################################################

    # Le noeud de test courant est initialise au noeud de depart
    goal_node = Node(goal)
    init_node = Node(init)    
    curr_node = init_node
    nb_elem = 0

    """ if not is_node_out_obstacles(tableMap, goal_node, True):
        raise DestBlockedError """

    if not tableMap.get_avoid():
        final_path = []
        final_path.append(goal_node.get_position())
        return final_path

    # Si l'arrivee est visible depuis le départ, on renvoie directement un chemin vers l'arrivée
    if goal_node.is_visible(curr_node, tableMap):
        return [goal_node.get_position()]
    
    curr_node.set_init_dist(0)
    curr_node.set_goal_dist(curr_node.dist_from_node(goal_node))
    curr_node.set_weight(0)
    goal_node.set_goal_dist(0)
    goal_node.set_init_dist(goal_node.dist_from_node(curr_node))
    isFirstAccurate = not is_node_out_obstacles(tableMap, curr_node, True)
    is_in_avoid_mode = tableMap.get_avoid()
    
    # Init liste du A*
    opened_list = PriorityQueue() ## La queue des noeuds à regarder
    opened_list.put((curr_node.get_weight(), nb_elem, curr_node))
    nb_elem += 1
    closed_list = [] ## La liste des noeuds déjà visités
    
    # Liaison des noeuds de depart et d'arrivee avec les noeuds de la carte
    best_node = None
    best_dist = 0
    for node in tableMap.get_node_list():
        node.set_weight(10000)
        goal_dist = node.dist_from_node(goal_node)

        # Lors d'un evitement on ne connecte le noeud d'arrive qu'aux noeud adjacent
        if is_in_avoid_mode and goal_dist < 600:
            node.add_link_node_list(goal_node)
            goal_node.add_link_node_list(node)

        # On connecte le noeud de départ
        init_dist = node.dist_from_node(curr_node)

        if best_node == None:
            best_node = node
            best_dist = init_dist

        if init_dist < 600:
            # On rajoute le noeud s'il est visible et hors des obstacles
            if curr_node.is_visible(node, tableMap) and is_node_out_obstacles(tableMap, node, False):
                curr_node.add_link_node_list(node)
                node.add_link_node_list(curr_node)

            # On garde le noeud même si la liaison traverse un obstacle si jamais on avait pas de meilleur noeud
            if is_node_out_obstacles(tableMap, node, False):
                if init_dist < best_dist:
                    best_node = node
                    best_dist = init_dist

    # Si la liste de connection du noeud de depart est vide on force la premiere connection avec le noeud le plus proche          
    if curr_node.get_link_node_list() == [] and best_node != None:
        best_node.set_goal_dist(best_node.dist_from_node(goal_node))
        best_node.update_node(curr_node)

#######################################################################
# PHASE DE RECHERCHE
#######################################################################

    begin = time.time()

    found = False

    # Boucle de parcours du graphe selon l'algo de A*
    while not opened_list.empty() and not found:
        # NB: A peu pres 1000 passages ici quand on ne trouve pas de path (depend de la map et des positions)
        if (time.time() - begin) > float(maxAstarTime):  # interruption de l'Astar
            raise TimeOutError
        curr_node = opened_list.get()[2]
        
        for neighbor in curr_node.get_link_node_list() :

            if not neighbor.is_visible(curr_node, tableMap):  # on garanti que le noeud est visible depuis curr_node
                continue
            """ if is_node_in_list(closed_list, neighbor):          # on garanti qu'il n'est pas dans la liste fermé
                continue """
            
            if neighbor.equals(goal_node):
                neighbor.update_node(curr_node)
                curr_node = neighbor
                found = not found 
                break

            neighbor.set_goal_dist(neighbor.dist_from_node(goal_node))        
            new_cost = curr_node.get_weight() + neighbor.get_goal_dist() + neighbor.dist_from_node(curr_node)

            """ in_open = is_node_in_list(opened_list, neighbor)
            in_close = is_node_in_list(closed_list, neighbor) """

            if neighbor.get_weight() == None or neighbor.get_weight() > new_cost:    
                neighbor.update_node(curr_node)
                opened_list.put((neighbor.get_weight(), nb_elem, neighbor))
                nb_elem += 1
        closed_list.append(curr_node)

#######################################################################
# POST TRAITEMENT DU PATH
#######################################################################

    if not found :
        raise PathNotFoundError

    # On récupère le chemin (à l'envers) (found_path = [goal, ..., init])
    found_path = []
    while curr_node.parent != None:
        found_path.append(curr_node)
        curr_node = curr_node.get_parent()
    if not curr_node.equals(init_node):
        found_path.append(curr_node)
    
    final_path = []
    nb_pts = len(found_path)

    # Sinon on parcours la liste à l'envers et supprime les points intermediaire si inutiles (autres points visibles)
    for i in range(nb_pts-1, 1, -1):
        if found_path[i].is_visible(found_path[i-2], tableMap):
            del found_path[i-1]
            continue
        if (i < nb_pts-1):
            final_path.append(found_path[i].get_position())
    if nb_pts > 1:
        final_path.append(found_path[1].get_position())
    final_path.append(found_path[0].get_position())

    return final_path