# -*- coding: utf-8 -*-

'''
View class of the GUI node
It contains all the display functions (and no logic)
'''
import os
PATH = os.path.dirname(os.path.abspath(__file__))

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut, QGridLayout, QPushButton, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer, QRect, QMetaObject,QRectF, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush

from .components.MatchBoard import MatchBoard

from math import atan2

'''
Default coordinate system (can be translated, rotated and scaled)
--------> x
|
|
|
˅
y

'''


SCREEN_MARGIN = 300

REFRESH_FREQ = 30  # Hz

class View(QMainWindow):
	def __init__(self, model):
		super().__init__()

		self.model = model


		self.setObjectName("Main Window")
		#self.resize(800, 600)

		self.mainLayoutWidget = QWidget()
		self.mainLayoutWidget.setObjectName("centralwidget")
		self.setCentralWidget(self.mainLayoutWidget)

		self.mainLayout = QVBoxLayout(self.mainLayoutWidget)
		self.mainLayout.setContentsMargins(0, 0, 0, 0)
		self.mainLayout.setSpacing(0)
		self.mainLayout.setObjectName("verticalLayout")


		# self.mainLayout = QGridLayout(self.mainLayoutWidget)
		# self.mainLayout.setGeometry(QRect(30, 20, 721, 521))
		# self.mainLayout.setContentsMargins(0, 0, 0, 0)
		# self.mainLayout.setObjectName("Main layout")

		# self.mainLayout.setColumnStretch(0, 5)
		# self.mainLayout.setColumnStretch(1, 1)
		# self.mainLayout.setRowStretch(0, 5)
		# self.mainLayout.setRowStretch(1, 1)

		# print("Rows : ", self.mainLayout.rowCount())
		# print("Columns : ", self.mainLayout.columnCount())
		# print("stretch : ", self.mainLayout.rowStretch(0))

		self.horizontalLayout = QHBoxLayout()
		self.horizontalLayout.setObjectName("horizontalLayout")

		self.matchBoard = MatchBoard()
		# self.matchBoard.setGeometry(QRect(30, 20, 721, 521))
		self.horizontalLayout.addWidget(self.matchBoard, stretch=4)



		self.rightGridLayout = QGridLayout(self.mainLayoutWidget)
		self.rightGridLayout.setGeometry(QRect(30, 20, 721, 521))
		self.rightGridLayout.setContentsMargins(0, 0, 0, 0)
		self.rightGridLayout.setObjectName("Main layout")

		self.pushButton_2 = QPushButton()
		self.pushButton_2.setObjectName("pushButton_2")
		self.pushButton_2.setText("Bonsoir")

		self.rightGridLayout.addWidget(self.pushButton_2)
		# self.rightGridLayout.addWidget(self.pushButton_2, 1, 0)
		# self.rightGridLayout.addWidget(self.pushButton_2, 0, 1)
		# self.rightGridLayout.addWidget(self.pushButton_2, 1, 1)


		self.horizontalLayout.addLayout(self.rightGridLayout, stretch=1)

		# self.horizontalLayout.setStretch(4, 1)


		self.mainLayout.addLayout(self.horizontalLayout, stretch=4)


		self.pushButton_3 = QPushButton()
		self.pushButton_3.setObjectName("pushButton_2")
		self.pushButton_3.setText("Change Orientation")
		self.pushButton_3.clicked.connect(self.switchOrientation)

		self.mainLayout.addWidget(self.pushButton_3, stretch=1)


		self.mainLayout.setStretch(2, 1)





		# key bindings
		self.shortcut_close = QShortcut(QKeySequence('Ctrl+C'), self)
		self.shortcut_close.activated.connect(self.closeApp)



		# self.timer = QTimer(self)
		# self.timer.timeout.connect(self.updateMatchBoard)
		# self.timer.start(1000/REFRESH_FREQ)


		self.matchBoardClickStatus = 0  # to detect a click event and treat it only once

		self.lastClickedPos = None
		self.lastReleasedPos = None

		self.isVerticalOrientation = True



	def closeApp(self):
		print("Closing main window")
		self.close()





	def setOrientation(self, isVertical):
		if isVertical:
			# self.setGeometry(0+SCREEN_MARGIN, 0+SCREEN_MARGIN,
			# 				 self.screenWidth-SCREEN_MARGIN, self.screenHeight-SCREEN_MARGIN)
			self.setGeometry(100, 100,
							 self.screenWidth/2, self.screenHeight-200)
			self.matchBoard.setVerticalOrientation()

		else:
			# self.setGeometry(0+SCREEN_MARGIN, 0+SCREEN_MARGIN,
			# 				 self.screenHeight-SCREEN_MARGIN, self.screenHeight-SCREEN_MARGIN)
			self.setGeometry(100, 100,
							 self.screenWidth-200, self.screenHeight-500)
			self.matchBoard.setHorizontalOrientation()


	def switchOrientation(self):

		self.isVerticalOrientation = not self.isVerticalOrientation
		self.setOrientation(self.isVerticalOrientation)


	def setupScreenDims(self, dims):
		self.screenWidth = dims.width()
		self.screenHeight = dims.height()



	
	def blinkLED(self, led, col):
		return

	def setLED(self, led, col):
		return


	def updateMatchBoard(self):
		'''All things to update '''

		for k in range(1):  # TODO : update all robots instead of just num 0

			with self.model.lock:
				pos = self.model.robot_positions[k]
			self.matchBoard.plotRobot(k, pos)  # id, [x, y, theta]


	def matchBoardClickOrder(self):
		'''Returns None if no order has been given, or the order only once'''


		if self.matchBoardClickStatus == 0 and self.matchBoard.isMatchBoardClicked():

			self.lastClickedPos = self.matchBoard.getLastClickedPosition()

			if self.lastClickedPos is None: # cancel, weird case
				print("ERROR : clicked position is None")

			self.matchBoardClickStatus = 1

			return None


		elif self.matchBoardClickStatus == 1 and not self.matchBoard.isMatchBoardClicked():

			self.lastReleasePos = self.matchBoard.getLastReleasedPosition()

			if self.lastReleasePos is None: # cancel, weird case
				print("ERROR : released position is None")

			self.matchBoardClickStatus = 0

			return (self.lastClickedPos[0], self.lastClickedPos[1] , atan2(self.lastReleasePos[1] - self.lastClickedPos[1], self.lastReleasePos[0] - self.lastClickedPos[0]))
			
		else:
			return None

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