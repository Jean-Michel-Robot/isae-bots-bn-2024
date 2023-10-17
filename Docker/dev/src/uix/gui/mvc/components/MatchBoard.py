# -*- coding: utf-8 -*-

'''
Board
'''

import os
PATH = os.path.dirname(os.path.abspath(__file__))

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut
from PyQt5.QtCore import Qt, QTimer, QRect, QMetaObject,QRectF, QPoint, QLine
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush, QTransform, QCursor, QPolygonF, QPainterPath, QMatrix3x3

import numpy as np
from numpy import pi, sqrt, cos, sin, arctan2


# CONSTANTS #

X_MARGIN = 0
Y_MARGIN = 0

XBORDERLEFT = 0
YBORDER = 0


# TODO : read from config file
ROBOT_WIDTH = 195
ROBOT_HEIGHT = 160

ROBOT_DIAG = np.linalg.norm([ROBOT_WIDTH, ROBOT_HEIGHT])
ROBOT_ANGLE = arctan2(ROBOT_HEIGHT, ROBOT_WIDTH)

CLICK_CIRCLE_RADIUS = 5

CLICK_LINE_RADIUS = 40
CLICK_LINE_WIDTH = 3


class MatchBoard(QWidget):

	matchBoardSizeFactor = 0.2
	matchBoardDims = (2000*matchBoardSizeFactor, 3000*matchBoardSizeFactor)  # px

	scale = 1

	def __init__(self):
		super().__init__()


		self.bg_image = QPixmap(os.path.join(PATH, '../../image_files/vinyle_2023.png'))
		self.bg_dims = (self.bg_image.size().width(), self.bg_image.size().height())


		self.bgOrientation = None

		# self.robot_rect = QRect(0, 0, 150, 100)  # TODO : robot dimensions
		self.robot_shape = QPolygonF([QPoint(0,0), QPoint(0,0), QPoint(0,0), QPoint(0,0)])
		self.robot_line = QLine(QPoint(0,0), QPoint(0,0))

		# init robot position (start pos)
		# TODO

		self.lastClickedPosition = None
		self.lastReleasedPosition = None
		self.matchBoardClicked = False

		self.painter = QPainter()
		self.painter.setRenderHint(QPainter.Antialiasing)


		# Creating the transformations for the map
		a = pi/180 * 10
		tx = 0
		ty = 0
		sx = 0.5
		sy = 0.5
		sina = sin(a)
		cosa = cos(a)

		translationTransform = QTransform(1, 0, 0, 1, tx, ty)
		rotationTransform = QTransform(cosa, sina, -sina, cosa, 0, 0)
		scalingTransform = QTransform(sx, 0, 0, sy, 0, 0)

		self.transform1 = QTransform()  # TODO : remove ?
		self.transform1 = translationTransform * scalingTransform * rotationTransform

		t = self.transform1
		self.matrix = np.array([[t.m11(), t.m12(), t.m13()], [t.m21(), t.m22(), t.m23()], [t.m31(), t.m32(), t.m33()]])


		translationTransform = QTransform(1, 0, 0, 1, 0, 0)
		rotationTransform = QTransform(cosa, sina, -sina, cosa, 0, 0)
		scalingTransform = QTransform(1/sx, 0, 0, 1/sy, 0, 0)

		self.transform2 = QTransform()  # TODO : remove ?
		self.transform1 = translationTransform * scalingTransform * rotationTransform
		t = self.transform2
		self.invMatrix = np.array([[t.m11(), t.m12(), t.m13()], [t.m21(), t.m22(), t.m23()], [t.m31(), t.m32(), t.m33()]])

	# def switchOrientation(self, orientation):
	# 	'''Called by button press'''

	# 	if self.bgOrientation == "vertical": self.setHorizontalOrientation()
	# 	elif self.bgOrientation == "horizontal": self.setVerticalOrientation()
	# 	else: print("ERROR : No orientation defined")

		self.test = False


	def setVerticalOrientation(self):

		self.matchBoardSizeFactor = 0.2
		self.matchBoardDims = (2000*self.matchBoardSizeFactor, 3000*self.matchBoardSizeFactor)

		self.setFixedSize(self.matchBoardDims[0], self.matchBoardDims[1])
		# if self.bgOrientation is not None: self.bg_image = self.bg_image.transformed(QTransform().rotate(-90))

		self.scale = 0.5

		self.bgOrientation = "vertical"


	def setHorizontalOrientation(self):

		self.matchBoardSizeFactor = 0.375
		self.matchBoardDims = (3000*self.matchBoardSizeFactor, 2000*self.matchBoardSizeFactor)

		self.setFixedSize(self.matchBoardDims[0], self.matchBoardDims[1])
		# if self.bgOrientation is not None: self.bg_image = self.bg_image.transformed(QTransform().rotate(90))

		self.scale = 1

		self.bgOrientation = "horizontal"




	def mouseMoveEvent(self, QMouseEvent):

		# inputPos = (QMouseEvent.pos().x(), QMouseEvent.pos().y())
		inputPos = np.array([QMouseEvent.pos().x(), QMouseEvent.pos().y(), 1])

		# print(self.matrix)
		# inputPos = self.matrix @ inputPos[:2,:2]
		# print(inputPos)

		self.clickOrderAngle = arctan2(inputPos[1] - self.clickCircleCenter.y(), inputPos[0] - self.clickCircleCenter.x())

			
	def mousePressEvent(self, QMouseEvent):

		# inputPos = (QMouseEvent.pos().x(), QMouseEvent.pos().y())
		inputPos = np.array([QMouseEvent.x(), QMouseEvent.y(), 1])

		# print(self.matrix)
		# mat = np.linalg.inv(self.matrix)
		# inputPos = np.dot(self.invMatrix, inputPos)[:2]  # invert matrix ?
		# print(inputPos)

		self.clickCircleCenter = QPoint(inputPos[0], inputPos[1])
		self.clickOrderAngle = 0  # as first position angle

		self.lastClickedPosition = self.toBoardPos(inputPos)
		self.matchBoardClicked = True

	def mouseReleaseEvent(self, QMouseEvent):

		inputPos = np.array([QMouseEvent.pos().x(), QMouseEvent.pos().y(), 1])

		# print(self.matrix)
		# mat = np.linalg.inv(self.matrix)
		# inputPos = np.dot(mat, inputPos)[:2]  # invert matrix ?
		# print(inputPos)

		
		self.lastReleasedPosition = self.toBoardPos(inputPos)
		self.matchBoardClicked = False

	def isMatchBoardClicked(self):return self.matchBoardClicked

	def getLastClickedPosition(self): return self.lastClickedPosition

	def getLastReleasedPosition(self): return self.lastReleasedPosition


	
	def paintEvent(self, event):

		self.painter.begin(self)

		# self.painter.setTransform(self.transform1)

		if not self.test:
			# self.painter.translate(QPoint(100,10))
			self.painter.rotate(10.0)  # can be used to flip the table (vertical or horizontal), it flips the whole coordinate system
			print(event)
			# self.painter.scale(0.5, 0.5)  # idem with scale, can be used to zoom in or out


		# if self.bgOrientation is not None:

		# 	if self.bgOrientation == "vertical":
		# 		self.painter.rotate(-45)
		# 		# self.bg_image = self.bg_image.transformed(QTransform().rotate(-90))
		# 	else:
		# 		self.painter.rotate(90)
		# 		self.bg_image = self.bg_image.transformed(QTransform().rotate(45))

		#self.painter.setFont(QFont('Open Sans', 12))
		# self.painter.drawText(event.rect(), Qt.AlignCenter, self.text)



		# vertical
		# self.painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, 500 - X_MARGIN, 750 - Y_MARGIN),
		# 				   self.bg_image)

		# horizontal
		self.painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, self.matchBoardDims[0] - X_MARGIN, self.matchBoardDims[1] - Y_MARGIN),
						   self.bg_image)



		#### CLICK ORDER ####
		if self.lastClickedPosition is not None:

			self.painter.setPen(QPen(QColor(57, 173, 240), CLICK_LINE_WIDTH, Qt.SolidLine))
			self.painter.drawLine(self.clickCircleCenter, self.clickCircleCenter + QPoint( int(CLICK_LINE_RADIUS*cos(self.clickOrderAngle)), int(CLICK_LINE_RADIUS*sin(self.clickOrderAngle)) ))

			self.painter.setPen(QPen(Qt.black, 1, Qt.SolidLine))
			self.painter.setBrush(QColor(57, 173, 240))

			self.painter.drawEllipse(self.clickCircleCenter, CLICK_CIRCLE_RADIUS, CLICK_CIRCLE_RADIUS)

		

		#### PAINT ROBOT ###

		self.painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
		brush = QBrush(QColor(200, 20, 20), Qt.SolidPattern)

		path = QPainterPath()
		path.addPolygon(self.robot_shape)

		self.painter.drawPolygon(self.robot_shape)
		self.painter.fillPath(path, brush)

		self.painter.drawLine(self.robot_line)



		self.painter.end()



	def blinkLED(self, led, col):
		return

	def setLED(self, led, col):
		return


	def toPixelPos(self, pos):
		'''Takes a position in mm (0 to 3000) and turns it into a pixel pos'''
		return (self.matchBoardSizeFactor*(pos[0] + XBORDERLEFT), self.matchBoardDims[1] - self.matchBoardSizeFactor*(pos[1] + YBORDER))

	def toBoardPos(self, pos):
		'''Inverse operation'''
		return (pos[0]/self.matchBoardSizeFactor - XBORDERLEFT, YBORDER - (pos[1] - self.matchBoardDims[1])/self.matchBoardSizeFactor)


	def plotRobot(self, k, pos):

		# TODO : multiple robot rects
		
		# update robot k
		(x, y, theta) = pos

		pointNb = 0
		for i in range (-1, 2, 2):
			for j in range (-1, 2, 2):
				anglePos = (x + ROBOT_DIAG/2*cos(theta - i*pi/2 + j*ROBOT_ANGLE),
							y + ROBOT_DIAG/2*sin(theta - i*pi/2 + j*ROBOT_ANGLE))

				# conversion to pixel points
				anglePos_px = self.toPixelPos(anglePos)

				self.robot_shape.replace(pointNb, QPoint(int(anglePos_px[0]), int(anglePos_px[1])))
				pointNb += 1

		centerPos_px = self.toPixelPos( (x, y) )

		# Coordonnees du milieu du cote avant du robot
		centerLinePos = (x + ROBOT_HEIGHT/2 * cos(theta),
						 y + ROBOT_HEIGHT/2 * sin(theta))
		
		centerLinePos_px = self.toPixelPos(centerLinePos)

		self.robot_line.setPoints(QPoint(int(centerPos_px[0]), int(centerPos_px[1])),
								  QPoint(int(centerLinePos_px[0]), int(centerLinePos_px[1])))



		self.update()




# # don't auto scale when drag app to a different monitor.
# # QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

# def main():
# 	app = QApplication(sys.argv)
# 	view = View()
# 	view.show()
# 	app.exec_()

# 	print("End")

# try:
#     sys.exit(app.exec_())
# except SystemExit:
#     print('Closing Window...')