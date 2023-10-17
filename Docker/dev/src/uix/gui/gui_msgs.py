# -*- coding: utf-8 -*-

from gui_utils import log_errs, log_fatal
import rospy

from std_msgs.msg      import Empty, Int16, Int16MultiArray
from geometry_msgs.msg import Quaternion, Pose2D




def init_msgs(self):

    # Subscribers
    self.start_sub = rospy.Subscriber("/game/start", Int16, self.update_start)
    self.color_sub = rospy.Subscriber("/game/color", Int16, self.update_color)

    self.pos_sub = rospy.Subscriber("/current_position", Pose2D, self.update_position)




def update_start(self, msg):

    if msg.data == 1:
        self.model.isMatchStarted = True



def update_color(self, msg):

    if msg.data != 0 and msg.data != 1:
        log_fatal(f"{msg.data} is not a valid color value, ignoring color update")

    self.controller.set_side(msg.data)


def update_position(self, msg):

    try:
        self.controller.set_robot_pos(0, msg.x, msg.y, msg.theta)
    except AttributeError:
        print("GuiNode class not initialized yet")



clickOrder_pub = rospy.Publisher("/nextPositionTeensy", Quaternion, queue_size=10, latch=False)

def send_click_order(clickOrder):

    clickOrder_pub.publish(Quaternion(clickOrder[0], clickOrder[1] , clickOrder[2], 0))
