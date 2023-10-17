#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@file: disp_gain.py
@status: OK
"""

#################################################################
#																#
# 							IMPORTS 							#
#																#
#################################################################

# import utils
from disp_utils import LOG_INFO
from disp_utils import READER
# import comm
from disp_comm import pub_speed, pub_gains, pub_gains_motor, pub_dyn1, pub_dyn2

#################################################################
#																#
# 						  DEFINITIONS 							#
#																#
#################################################################

def init_gain(displacementNode):
    """Initialisation de la Teensy."""

    ## Pointeur sur le DisplacementNode
    global p_dn
    p_dn = displacementNode

#######################################################################
# LECTURE DES INFOS DANS .ini
#######################################################################
    LOG_INFO("Teensy setup start.")

    # Lecture des gains
    P = READER.getfloat("Asserv", "gain_distance_p")
    I = READER.getfloat("Asserv", "gain_distance_i")
    D = READER.getfloat("Asserv", "gain_distance_d")

    P_a = READER.getfloat("Asserv", "gain_angle_p")
    I_a = READER.getfloat("Asserv", "gain_angle_i")
    D_a = READER.getfloat("Asserv", "gain_angle_d")
    gains = [P, I, D, P_a, I_a, D_a]

    C_m = READER.getfloat("Asserv", "gain_moteur_c")
    P_m = READER.getfloat("Asserv", "gain_moteur_p")
    I_m = READER.getfloat("Asserv", "gain_moteur_i")
    D_m = READER.getfloat("Asserv", "gain_moteur_d")
    gainsMotor = [C_m, P_m, I_m, D_m]

    # Lecture parametres dyn
    accLin = READER.getfloat("Asserv", "ramp_acc_lin")
    decLin = READER.getfloat("Asserv", "ramp_dec_lin")
    decMaxLin = READER.getfloat("Asserv", "ramp_decMax_lin")
    accRot = READER.getfloat("Asserv", "ramp_acc_rot")
    decRot = READER.getfloat("Asserv", "ramp_dec_rot")
    accLinSmooth = READER.getfloat("Asserv", "ramp_acc_lin_smooth")
    decLinSmooth = READER.getfloat("Asserv", "ramp_dec_lin_smooth")
    decMaxLinSmooth = READER.getfloat("Asserv", "ramp_decMax_lin_smooth")
    dynamicParams1 = [accLin, decMaxLin, decLin, accRot, decRot]
    dynamicParams2 = [accLin, decMaxLin, decLin, accLinSmooth, decMaxLinSmooth, decLinSmooth, accRot, decRot]

    # Vitesses max (Lin et Rot)
    maxSpeedLin = READER.getfloat("Asserv", "speed_lin")
    maxSpeedRot = READER.getfloat("Asserv", "speed_rot")
    
    p_dn.speedLin = maxSpeedLin
    p_dn.speedRot = maxSpeedRot
    p_dn.maxSpeedLin = maxSpeedLin
    p_dn.maxSpeedRot = maxSpeedRot

#######################################################################
# PUBLICATION DES INFOS
#######################################################################

    pub_speed.publish(data=[maxSpeedLin, maxSpeedRot])

    ## Publication des gains
    #pub_gains.publish(data=gains)
    #pub_gains_motor.publish(data=gainsMotor)
    
    ## Publication param dyn
    #pub_dyn1.publish(data=dynamicParams1)
    #pub_dyn2.publish(data=dynamicParams2)

    LOG_INFO("Teensy setup done.")