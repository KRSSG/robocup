from tactic import Tactic
import time
import sys
from math import *

sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
import obstacle

import sKickToPoint
import sGoToBall
import sDribbleTurn
import numpy as np


KICK_RANGE_THRESH = MAX_DRIBBLE_R   #ASK
THRES  = 0.8
THETA_THRESH = 0.005
TURNING_THRESH = 10
# k=k2=50000 no threshold
THRESH=15*pi/180
BOT_OPP_DIST_THRESH =  500
from time import time

ROWS                                    =  33
COLS                                    =  22
OPTIMAL_DISTANCE                        =  3.0*HALF_FIELD_MAXX/5
BISECTOR_CONST                          =  70
OUR_BOT_APPROACHABILITY_CONST           =  35 
OPTIMAL_DISTANCE_CONST                  =  10
AWAY_BOT_APP_GWT                        =  30

k=k2=20*pi/180

class TPosition(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX, OUR_GOAL_MINY)

    def execute(self, state, pub):

        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        DRIBBLER_BALL_THRESH = 110
        import sGoToBall
        for i in xrange(3):
            sGoToBall.execute(self.sParam, state, i, pub) 
        #ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        # DRIBBLER_BALL_THRESH = 110
        # import sGoToBall
        # for i in xrange(6):
        #     botPos = Vector2D(int(state.homePos[i].x), int(state.homePos[i].y))
        #     if ballPos.dist(botPos) <= DRIBBLER_BALL_THRESH :    
        #         #sGoToPoint.execute(self.sParam, state, 1, pub) 
        #         self.sParam.KickToPointP.power=7
        #         print(ballPos.dist(botPos),"dribbler on",i, ballPos.x, ballPos.y , botPos.x , botPos.y)
        #         for _ in xrange(5):
        #             print("____KICK___")
        #             self.sParam.KickToPointP.x = state.homePos[1].x
        #             self.sParam.KickToPointP.y = state.homePos[1].y
        #             sKickToPoint.execute(self.sParam, state, i, pub) 
        #     else:
        #         print("dist  123  ", ballPos.dist(botPos), i, ballPos.x, ballPos.y , botPos.x , botPos.y)
        #         for _ in xrange(5):
        #             print("__GOTOBALL__")
        #             sGoToBall.execute(self.sParam, state, i, pub) 
        

        # import sTurnToPoint
        
        # # # i = 0
        # for i in xrange(1,2):

        #     finalslope =  ballPos.angle(Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)))

        #     ob =Vector2D()
        #     angletoturn = ob.normalizeAngle(finalslope - state.homePos[i].theta)


        #     print(angletoturn,"ans",i)
        #     if fabs(angletoturn) > SATISFIABLE_THETA*4 : 

        #         #print ("aligning\n", i)
        #         self.sParam.TurnToPointP.x = ballPos.x
        #         self.sParam.TurnToPointP.y = ballPos.y
        #         self.sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA/ 2.0
        #         print("before turn", i )

        #             #print("__TURNING TO ANGLE___")
        #         sTurnToPoint.execute(self.sParam, state, i, pub)
                    
        #     else:
        #         import sDribble

        #         sDribble.execute(self.sParam,state,i,pub)
                


       

    def isComplete(self, state):
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[i]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        # No parameter to update here
        pass
