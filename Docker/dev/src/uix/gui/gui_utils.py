# -*- coding: utf-8 -*-


import rospy
import gui_const

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################



#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

class Color():
	BLACK = '\033[30m'
	RED = '\033[31m'
	GREEN = '\033[32m'
	YELLOW = '\033[33m'
	BLUE = '\033[34m'
	MAGENTA = '\033[35m'
	CYAN = '\033[36m'
	WHITE = '\033[37m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'
	RESET = '\033[0m'

def log_info(log):
    """
    Print standard logs
    """
    rospy.loginfo(f"{Color.WHITE}[{gui_const._NODENAME_}] {log}{Color.RESET}")


def log_warn(log):
    """
    Print warning logs
    """
    rospy.logwarn(f"{Color.YELLOW}[{gui_const._NODENAME_}] {log}{Color.RESET}")


def log_errs(log):
    """
    Print errors logs (errors concerning actions, not critical)
    """
    rospy.logerr(f"{Color.RED}[{gui_const._NODENAME_}] {log}{Color.RESET}")


def log_fatal(log):
    """
    Print fatal error logs (critical errors, not concerning actions and never supposed to happen)
    """
    rospy.logfatal(f"{Color.BOLD}{Color.UNDERLINE}{Color.RED}[{gui_const._NODENAME_}] {log}{Color.RESET}")