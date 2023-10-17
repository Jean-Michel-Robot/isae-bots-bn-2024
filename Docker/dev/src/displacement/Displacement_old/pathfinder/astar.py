#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

from node import Node
from exceptions import PathNotFoundError, TimeOutError

### CONSTANTES ########################################################
X_THRESHOLD = 5     # 5mm
Y_THRESHOLD = 5     # 5mm
#######################################################################

def isNodeInList(list, node):
    #Renvoie Vrai et le noeud si le noeud est dans la liste
    for testNode in list:
        if testNode.equals(node):
            return True
    return False

def findBestNode(open_list):
    #Renvoie le meilleur noeud de la liste et le supprime
    if open_list == []:
        raise PathNotFoundError()
    best=0
    for i in range(len(open_list)):
        if open_list[i].getWeight()<open_list[best].getWeight():
            best=i
    return open_list.pop(best)

def isNodeOutObstacles(tableMap, node, isFirst):
    #Teste si le noeud est en dehors des obstacles
    if not tableMap.getAvoid():
        return True
    if not isFirst:
        return True
    for obstacle in tableMap.getObstacleList():
        if obstacle.isNodeIn(node):
            return False


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
    goalNode = Node(goal)
    currNode = Node(init)    
    currNode.setInitDist(0)
    isFirstAccurate[0] = not isNodeOutObstacles(tableMap, currNode, True)
    
    # Init liste du A*
    openedList = []
    closedList = [currNode]
    
    # Liaison des noeuds de depart et d'arrivee avec les noeuds de la carte
    bestNode = None
    bestDist = 0
    for node in tableMap.getNodeList():
        goalDist = node.distFromNode(goalNode)

        # Lors d'un evitement on ne connecte le noeud d'arrive qu'aux noeud adjacent
        if tableMap.getAvoid() and goalDist < 200:  
            node.addLinkNodeList(goalNode)
        else:
            node.addLinkNodeList(goalNode)

        # On connecte le noeud de départ
        initDist = node.distFromNode(currNode)
        if initDist < 1500:
            # On rajoute le noeud s'il est visible et hors des obstacles
            if currNode.isVisible(node, tableMap) and isNodeOutObstacles(tableMap, node, False):
                currNode.addLinkNodeList(node)

            # On sauve le noeud le plus proche, hors des obstacles, 
            # meme si la liaison traverse un obstacle
            if isNodeOutObstacles(tableMap, node, False):
                if bestNode == None or initDist < bestDist:
                    bestNode = node
                    bestDist = initDist

    # Si la liste de connection du noeud de depart est vide on force la premiere connection avec le noeud le plus proche          
    if currNode.getLinkNodeList() == []:
        bestNode.setGoalDist(bestNode.distFromNode(goalNode))
        bestNode.updateNode(currNode)
        closedList.append(bestNode)
        currNode = bestNode

#######################################################################
# PHASE DE RECHERCHE
#######################################################################

    begin = time.time()
    
    # Boucle de parcours du graphe selon l'algo de A*
    while not currNode.equals(goalNode):
        # NB: A peu pres 1000 passages ici quand on ne trouve pas de path (depend de la map et des positions)
        if time.time() - begin > maxAstarTime:  # interruption de l'Astar
            raise TimeOutError
        
        for testNode in currNode.getLinkNodeList() :
            testNode.setGoalDist(testNode.distFromNode(goalNode))

            if not testNode.isVisible(currNode, tableMap):  # on garanti que le noeud est visible depuis currNode
                continue
            if isNodeInList(closedList, testNode):          # on garanti qu'il n'est pas dans la liste fermé
                continue
            if isNodeInList(openedList, testNode):          # s'il est dans la liste ouverte mais de meilleure qualite
                if testNode.getWeight() > testNode.calculWeight(currNode): 
                    testNode.updateNode(currNode)
            else:
                testNode.updateNode(currNode)
                openedList.append(testNode)
        
        currNode = findBestNode(openedList)                    
        closedList.append(currNode)

#######################################################################
# POST TRAITEMENT DU PATH
#######################################################################

    # On récupère le chemin (à l'envers)
    foundPath = []
    testNode = currNode
    while testNode.parent != None:
        foundPath.append(testNode)
        testNode = testNode.getParent()
    foundPath.append(testNode)
    
    finalPath = []
    nbPts = len(foundPath)
    
    # Si l'arrivee est visible du depart
    if foundPath[0].isVisible(foundPath[-1], tableMap):
        finalPath.append(foundPath[0].getPosition())    
        return finalPath
    
    # Sinon on parcours la liste à l'envers et supprime les points intermediaire si inutiles (autres points visibles)
    for i in range(nbPts-1, 1, -1):
        if foundPath[i].isVisible(foundPath[i-2], tableMap):
            del foundPath[i-1]
            continue
        if (i < nbPts-1):
            finalPath.append(foundPath[i].getPosition())

    finalPath.append(foundPath[1].getPosition())
    finalPath.append(foundPath[0].getPosition())

    # Si le premier point est juste a cote du robot, on l'enleve
    x, y = finalPath[0][0], finalPath[0][1]
    while abs(x-init[0]) < X_THRESHOLD and abs(y-init[1]) < Y_THRESHOLD:
        finalPath.pop(0)

    return finalPath