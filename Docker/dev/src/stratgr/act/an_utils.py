#!/usr/bin/env python
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

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import rospy
from an_const import SIMULATION, NODE_NAME, COLOR, ONE_PI

#################################################################
#                                                               #
#                          CONSTANTS                            #
#                                                               #
#################################################################

def log_info(log):
    """
    Print standard logs.
    """
    rospy.loginfo(NODE_NAME + log)


def log_warn(log):
    """
    Print warning logs.
    """
    rospy.logwarn(NODE_NAME + log)

def log_errs(log):
    """
    Print errors logs.
    """
    rospy.logerr(NODE_NAME + log)

def patch_frame_br(x, y, theta, color):
    if color == 0:
        return x, y, theta
    return 2000-x, y, theta+ONE_PI # Si la symétrie est selon l'axe y.
    # return x, 3000-y, -theta # Si la symétrie est selon l'axe x.

#################################################################
# Colors gestion												#
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

color_dict = {'n':Color.BLACK, 'r':Color.RED, 'g':Color.GREEN, 'y':Color.YELLOW, 'b':Color.BLUE, 
			 'm':Color.MAGENTA, 'c':Color.CYAN, 'w':Color.WHITE}

#################################################################
# Debug functions												#
#################################################################

# Enable or disable debug prints
debug_prints = True  

# Debug print function
def debug_print(msg, format):
	
	# If debug prints are disabled, quit
	if not debug_prints: return

	# If no color was specified, error & quit
	if len(format) == 0:
		print(Color.RED + "Wrong debug_print color" + Color.RESET)
		return

	print_string = ''
	color = format[0]

	if len(format[1:]) > 0:
		shape = format[1:]
		if shape == '*': 
			print_string += Color.BOLD
		elif shape == '-': 
			print_string += Color.UNDERLINE
		elif shape == '*-': 
			print_string += Color.BOLD + Color.UNDERLINE
	
	try:
		print_string += color_dict[color] + str(msg) + Color.RESET
		print(print_string)
	except KeyError:
		print(Color.RED + "Wrong debugPrint color" + Color.RESET)
		return