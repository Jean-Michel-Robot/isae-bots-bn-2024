# -*- coding: utf-8 -*-

# pyright: reportMissingImports=false

from numpy import pi, cos, sin, tan, arccos, arcsin, arctan, linalg
from mttkinter import mtTkinter as tk


def deg(a): return a*180/pi

def X(self, x): return self.DIM*(x + self.XBORDERLEFT)
def Y(self, y): return self.HEIGHT - self.DIM*(y + self.YBORDER)

# The other way around
def Xinput(self, x): return x/self.DIM - self.XBORDERLEFT
def Yinput(self, y): return self.YBORDER - (y - self.HEIGHT)/self.DIM

def dist(pos1, pos2): return linalg.norm([pos2[1] - pos1[1], pos2[0] - pos1[0]])

def update_path(self,msg):
    with self.lock:
        if msg.data is None: return
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






# def create_bouee_number(self, x, y, num, canvas):
#     x0 = self.DIM*x
#     y0 = self.HEIGHT - self.DIM*y
#     canvas.create_text(x0, y0, fill="black", font=('Sans', "12", 'bold'), text=str(num))


# def update_bouees(self,msg):
#     changes = msg.data
#     if changes[0] == 0:
#         self.canvas.delete("bouee_circle"+str(changes[1]))




def create_sample(self, x, y, theta, r, id_sample, color):
    coords = []
    for i in range(6):
        coords.append(X(self, x + r*cos(i*pi/3 + theta)))
        coords.append(Y(self, y + r*sin(i*pi/3 + theta)))
    self.canvas.create_polygon(coords, fill=color, width=0, tag='sample'+str(id_sample))



def update_samples(self, msg):
    with self.lock:
        #TODO : fct du subscriber
        id_sample = msg.data
        self.canvas.delete('sample'+str(id_sample))

def create_excavation_square(self, x, y, l , w, color):
    coords = []
    coords.append(X(self, x + w/2))
    coords.append(Y(self, y + l/2 ))
    coords.append(X(self, x - w/2))
    coords.append(Y(self, y + l/2 ))
    coords.append(X(self, x - w/2))
    coords.append(Y(self, y - l/2 ))
    coords.append(X(self, x + w/2))
    coords.append(Y(self, y - l/2 ))
    self.canvas.create_polygon(coords, fill=color, width = 0, tag='excavation_square')



colorDict = {-1:"black", 0:"yellow", 1:"purple", 2:"red"}

def plot_excavation_squares(self):

    self.canvas.delete("excavation_square")

    for k in range(10):
        if self.squareInfo[k] == 1: color = colorDict[self.squareLayout[k]]
        else: color = "brown"
        self.create_excavation_square(self.excavationSquarePos_x, self.excavationSquarePos_y[k], self.EXCAVATION_SQUARE_SIDE, self.EXCAVATION_SQUARE_WIDTH, color)
    
    self.isExcavationSquareChanged = False




def create_square_indicator(self, x, y, r, color):
    x0 = X(self, x - r)
    y0 = Y(self, y - r)
    x1 = X(self, x + r)
    y1 = Y(self, y + r)
    self.canvas.create_oval(x0, y0, x1, y1, fill=color, width=0, tag='square_indicator')


def plot_square_indicators(self):

    self.canvas.delete("square_indicator")

    for k in range(10):
        color = colorDict[self.squareLayout[k]]
        self.create_square_indicator(2060, self.excavationSquarePos_y[k], 20, color)