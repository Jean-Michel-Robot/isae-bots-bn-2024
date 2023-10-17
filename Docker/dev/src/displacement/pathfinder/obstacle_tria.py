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
@file: obstacle_tria.py
@status: OK
"""

class ObstacleTria:
    
    """
    Implemente un obstacle triangle rectangle sur la map.
    
    L'obstacle est represente par : 
    - (x,y) : la position du sommet rectangle
    - b     : la base (negative si vers la gauche)
    - h     : la hauteur (negative si vers le bas).
    """

    def __init__(self,p1,p2,p3):
        """Initialization of obstacle."""       
        self.name = "T"    # Type d'obstacle - triangle rectangle
        self.p1 = p1        # point 1 sous la forme (x1,y1)
        self.p2 = p2        # point 2 sous la forme (x2,y2)
        self.p3 = p3        # point 3 sous la forme (x3,y3)
        
    def get_p1(self):
        return self.p1
    
    def get_p2(self):
        return self.p2
    
    def get_p3(self):
        return self.p3
    
    def get_name(self):
        return self.name

    def is_node_in(self, node):
        x = node.get_x()
        y = node.get_y()

        ## Calcul des produits vectoriels 
        a = (self.p2[0]-x)*(self.p3[1]-y) - (self.p2[1]-y)*(self.p3[0]-x)
        b = (self.p3[0]-x)*(self.p1[1]-y) - (self.p3[1]-y)*(self.p1[0]-x)
        c = (self.p1[0]-x)*(self.p2[1]-y) - (self.p1[1]-y)*(self.p2[0]-x)

        ## On verifie que les angles ont le meme signe
        if a*b >= 0 and a*c >= 0 and b*c >= 0:
            return True
        return False

    