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
#
# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import time
from dn_comm  import next_action_pub, stop_IT, take_cakes_pub, deposit_cakes_pub, take_cherries_pub, score_pub, stage_pub, end_pub
from dn_utils import log_info, LIST_OF_ACTIONS, ACTIONS_SCORE

#################################################################
#                                                               #
#                            INIT                               #
#                                                               #
#################################################################

p_dn = None

def init_strats(dn):
    global p_dn
    p_dn = dn

#################################################################
#                                                               #
#                           STRATS                              #
#                                                               #
#################################################################

def test_strat():
    """
    DN Strat: homologation
    
    Note: Define below the actions of this strategy 
        -
        - 11 5
        -
    """
    time.sleep(0.01)

    #if p_dn is None: return  # safety if the function is called before DEC node init TODO : not supposed to happen ?

    if p_dn.go_park :
        p_dn.nb_actions_done[0] = 13

    if p_dn.nb_actions_done[0] == 0:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCherriesPerpendicular']
        take_cherries_pub.publish(0)
        log_info("Next action : Take Cherries Perpendicular")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 1:
        p_dn.curr_action = LIST_OF_ACTIONS['depositCherries']
        log_info("Next action : Deposit Cherries")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 2:
        p_dn.score += 10*ACTIONS_SCORE['cherryBucket']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(11)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 3:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(5)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 4:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 5:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(5)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 6:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 7:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(5)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 8:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 9:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(5)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 10:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 11:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(5)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 12:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 13:
        p_dn.score += 6*ACTIONS_SCORE['depositStage']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['park']
        log_info("Next action : Park")
        next_action_pub.publish(data=p_dn.curr_action)
        p_dn.score += ACTIONS_SCORE['parking']+ACTIONS_SCORE['funnyCounter']
        score_pub.publish(data=p_dn.score)
        return
    

    if p_dn.nb_actions_done[0] == 14:
        p_dn.nb_actions_done[0] = -1  # to prevent repeated end action
        log_info("End of strategy : TEST")
        stop_IT()
        return


def homologation():
    """
    DN Strat: tests (for all tests before the cup)
    
    Note: Define below the actions of this strategy
        - 
        -
        -
    """
    time.sleep(0.01)

    log_info("Entered homologation strat")

    if p_dn.nb_actions_done[0] == 0:
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(4)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 1:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 2:
        p_dn.score += 3*ACTIONS_SCORE['depositStage']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(0)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 3:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 4:
        p_dn.score += 3*ACTIONS_SCORE['depositStage']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['park']
        log_info("Next action : Park")
        next_action_pub.publish(data=p_dn.curr_action)
        p_dn.score += ACTIONS_SCORE['parking']+ACTIONS_SCORE['funnyCounter']
        score_pub.publish(data=p_dn.score)
        return

    if p_dn.nb_actions_done[0] == 5:
        end_pub.publish(data=1)
        p_dn.nb_actions_done[0] = -1  # to prevent repeated end action
        log_info("End of strategy : HOMOLOGATION")
        stop_IT()
        return

    return


def match_strat():
    """
    DN Strat: match (only for the cup)
    
    Note: Define below the actions of this strategy
        - 
    
        -
    """
    time.sleep(0.01)

    if p_dn.nb_actions_done[0] == 0:
        p_dn.curr_action = LIST_OF_ACTIONS['depositCherriesNear']
        log_info("Next action : Deposit Cherries")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 1:
        p_dn.score += 8*ACTIONS_SCORE['cherryBucket']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['pushCakes']
        deposit_cakes_pub.publish(2)
        log_info("Next action : Push Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return

    if p_dn.nb_actions_done[0] == 2:
        p_dn.score += 6*ACTIONS_SCORE['depositStage']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['takeCakes']
        take_cakes_pub.publish(12)
        log_info("Next action : Take Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 3:
        stage_pub.publish(data=-1)
        p_dn.curr_action = LIST_OF_ACTIONS['depositCakes']
        deposit_cakes_pub.publish(5)
        log_info("Next action : Deposit Cakes")
        next_action_pub.publish(data=p_dn.curr_action)
        return
    
    if p_dn.nb_actions_done[0] == 4:
        p_dn.score += 3*ACTIONS_SCORE['depositStage']
        score_pub.publish(data=p_dn.score)
        p_dn.curr_action = LIST_OF_ACTIONS['park']
        log_info("Next action : Park")
        next_action_pub.publish(data=p_dn.curr_action)
        p_dn.score += (ACTIONS_SCORE['parking']+ACTIONS_SCORE['funnyCounter'])
        score_pub.publish(data=p_dn.score)
        return

    if p_dn.nb_actions_done[0] == 4:
        p_dn.nb_actions_done[0] = -1  # to prevent repeated end action
        log_info("End of strategy : TEST")
        stop_IT()
        return

    pass
