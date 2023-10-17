# -*- coding: utf-8 -*-

'''
Modem class of the GUI node
It contains all the logic and behaviors
'''

from threading import Lock


class Model():

    lock = Lock()

    def __init__(self):
        

        #### BUTTONS AND STUFF ####

        self.verticalOrientation = True



        #### MATCH VARIABLES ####
        self.matchState = False
        self.side = None

        self.robot_positions = [[0, 0, 0]]*4  # 4 is the maximum number of robots
        self.nbRobots = 2


        print("Initialized Model")











    def set_robot_pos(self, id, x, y, theta):

        with self.lock:
            self.robot_positions[id][0] = x
            self.robot_positions[id][1] = y
            self.robot_positions[id][2] = theta


    def get_robot_pos(self, id):

        with self.lock:
            return self.robot_positions[id]