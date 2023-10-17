#!/usr/bin/env python
# pyright: reportMissingImports=false


import os
import sys
import time
import rospy
import i2c_lcd_driver
from subprocess   import Popen, PIPE
from std_msgs.msg import Int16


displayFrequency = 10

shortBlinkTime = 0.1
longBlinkTime = 0.4


nbCharsToDisplay = 10


def LOG_INFO(msg):
	rospy.loginfo("PTS "+ msg)

def handler(rcv_sig, frame):
	"""Force the node to quit on SIGINT, avoid escalating to SIGTERM."""
	LOG_INFO("Node forced to terminate...")
	rospy.signal_shutdown(rcv_sig)
	sys.exit()


class PointsNode:

	blinkingBackground = True  # choose false to deactivate


	def __init__(self):    
		self.subScore = rospy.Subscriber("/game/score", Int16, self.cbScore)
		self.lcd = i2c_lcd_driver.lcd()
		self.lcd.lcd_clear()
		self.lcd.lcd_display_string("SCORE : 0", line=1)


		rospy.loginfo("LCD Node Initialized")


	def cbScore(self, msg):
		self.lcd.lcd_clear()
		self.lcd.lcd_display_string("SCORE : " + str(msg.data), line=1)



### LAUNCH ###

if __name__ == '__main__':
	rospy.init_node('points_node')
	points = PointsNode()
	rospy.spin()
