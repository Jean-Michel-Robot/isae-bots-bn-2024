# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

#######################################################################
#																      #
# 							    IMPORTS 						      #
#																      #
#######################################################################

from numpy import pi, cos, sin, tan, arccos, arcsin, arctan, linalg
from mttkinter import mtTkinter as tk

from interface_utils import X, Y, Xinput, Yinput, deg, dist

#######################################################################
#																      #
# 							PATH MANAGEMENT						      #
#																      #
#######################################################################

def update_path(self,msg):
    with self.lock:
        if msg.data is None: 
            return
        pathReceived = list(msg.data)
        if len(pathReceived) > 0:
            self.finalCap = pathReceived.pop(-1)  # on enlève le cap final et on le store pour pouvoir l'afficher
            self.msgPath = pathReceived
            self.isNewPathReceived = True

def create_path_circle(self, x, y, r, canvas):
    x0 = X(self, x - r)
    y0 = Y(self, y - r)
    x1 = X(self, x + r)
    y1 = Y(self, y + r)
    canvas.create_oval(x0, y0, x1, y1, fill='yellow', tag='path_circle')

def create_path_line(self, x0, y0, x1, y1, canvas):
    x2 = X(self, x0)
    y2 = Y(self, y0)
    x3 = X(self, x1)
    y3 = Y(self, y1)
    canvas.create_line(x2, y2, x3, y3, fill = 'yellow', width = self.PATHWIDTH, tag = "path_line")

def create_path_arrow(self, xf, yf, finalCap, canvas):
    x2 = X(self, xf)
    y2 = Y(self, yf)
    x3 = X(self, xf + self.PATHARROWLENGTH*cos(finalCap))
    y3 = Y(self, yf + self.PATHARROWLENGTH*sin(finalCap))
    canvas.create_line(x2, y2, x3, y3, fill = 'purple', arrow=tk.LAST, arrowshape=(8,10,6), width = self.PATHWIDTH*2/3, tag = "path_line")

def plot_path(self,msg,msgPosition):
    '''Un path reçu est de la forme [x0, y0, x1, y1, ...]'''
    with self.lock:

        self.canvas.delete("path_circle")
        self.canvas.delete("path_line")
        #self.canvas.delete("vision_angles")

        l = len(msg)//2
        print("Longueur du path : {}".format(2*l))

        # 1er deplacement
        self.create_path_line(msgPosition.x, msgPosition.y, msg[0], msg[1], self.canvas)
        self.create_path_circle(msgPosition.x, msgPosition.y, self.PATHCIRCLEWIDTH, self.canvas)

        # reste des deplacements
        for k in range (l):
            if k < l-1:
                self.create_path_line(msg[2*k], msg[2*k+1], msg[2*k+2], msg[2*k+3], self.canvas)
            self.create_path_circle(msg[2*k], msg[2*k+1], self.PATHCIRCLEWIDTH, self.canvas)

        # fleche du dernier cap
        self.create_path_arrow(msg[2*l-2], msg[2*l-1], self.finalCap, self.canvas)
        self.isNewPathReceived = False


#######################################################################
#																      #
# 							GRID MANAGEMENT						      #
#																      #
#######################################################################


def update_grid(self,msg):
    with self.lock:
        if msg.data is None:
            return
        if list(msg.data) == []:
            return
        self.nodeGrid = list(msg.data)
        self.isGridDefined = True

def create_grid_circle(self, x, y, r, canvas):
    x0 = X(self, x - r)
    y0 = Y(self, y - r)
    x1 = X(self, x + r)
    y1 = Y(self, y + r)
    canvas.create_oval(x0, y0, x1, y1, fill='grey', tag='grid_circle')
        
def plot_grid(self,msg):
    with self.lock:
        self.canvas.delete("grid_circle")

        for i in range(len(msg) // 2):
            self.create_grid_circle(msg[2*i], msg[2*i+1], 5, self.canvas)
        