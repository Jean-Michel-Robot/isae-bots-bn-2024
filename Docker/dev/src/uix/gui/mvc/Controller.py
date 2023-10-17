# -*- coding: utf-8 -*-

'''
Controller class of the GUI node
It links the model, the view and the ROS node together
'''

from PyQt5.QtCore import QTimer

from geometry_msgs.msg import Quaternion, Pose2D

from gui_msgs import send_click_order


class Controller():

	def __init__(self, model, view):
		

		self.model = model
		self.view = view

		self.view.matchBoard.mousePressEvent

		self.timer = QTimer(self.view)
		self.timer.timeout.connect(self.refreshMatchView)
		self.timer.start(int(1000/30))



	def test(self):
		print("hello")


	def set_match_state(self, matchState):
		if matchState == self.model.matchState:
			self.view.blinkLED('matchLED', 'r')
			return

		self.model.matchState = matchState
		self.view.setLED('matchLED', 'g')


	def set_side(self, side):
		if side == self.model.side:
			self.view.blinkLED('sideLED', 'r')
			return

		self.model.side = side
		self.view.setLED('sideLED', 'g')


	def set_robot_pos(self, id, x, y, theta):
		pos_r = self.model.robot_positions[id]

		if pos_r is None:
			print(f"ERROR : robot with id {id} has no initialized position")
			return

		if (x, y, theta) == (pos_r[0], pos_r[1], pos_r[2]):
			return
			
		self.model.set_robot_pos(id, x, y, theta)

		# print(id, x, y, theta)

		# TODO : could check is robot_pos changed or not


	def refreshMatchView(self):
		'''Periodically for the matchBoard'''
		'''
		If a mutex is not available in read, we pass (it's gonna be read on the next refresh)
		If a mutex is not available in write, we wait (don't want to miss a write)
		'''

		self.view.updateMatchBoard()

		clickOrder = self.view.matchBoardClickOrder()
		if clickOrder is not None:
			send_click_order(clickOrder)
