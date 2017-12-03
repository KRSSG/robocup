from tactic import Tactic
import time
import sys
from math import *
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
from config import *
from numpy import array, inf

from geometry import * 
from math import *
import skills_union
import obstacle
import sGoToPoint
import sTurnToPoint

ANGLE_FACTOR=0.8   #close to the bot to mark orientation
DISTANCE_FACTOR=0.1

WEIGHT_IN_BALL_DIR=0.8

class TMark(Tactic):
    def __init__(self, bot_id, state,  param=None):
        super(TMark, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()


    def execute(self, state, pub, bot_to_mark = -1):
        self.bot_to_mark = bot_to_mark
        print("\n\n\n\nme"+str(self.bot_id)+" marking bot "+str(bot_to_mark)+"\n\n\n\n")
        ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        mark_bot_pos=Vector2D(int(state.awayPos[self.bot_to_mark].x), int(state.awayPos[self.bot_to_mark].y))
        mark_bot_angle=mark_bot_pos.normalizeAngle(state.awayPos[self.bot_to_mark].theta-ballPos.angle(mark_bot_pos))
        #destination_toward_attacker=Vector2D(int(mark_bot_pos.x*DISTANCE_FACTOR+ballPos.x*(1-DISTANCE_FACTOR)), int(mark_bot_pos.y*DISTANCE_FACTOR+ballPos.y*(1-DISTANCE_FACTOR)))
        destination_toward_attacker_angle=0
        
        best_angle=ANGLE_FACTOR*mark_bot_angle+ballPos.angle(mark_bot_pos)
        #if(fabs(mark_bot_angle- destination_toward_attacker_angle)>pi):
        #    best_angle=mark_bot_pos.normalizeAngle(best_angle+pi)
        #destination_toward_orientation=Vector2D(int(mark_bot_pos.x+DISTANCE_ORIENTATION*cos(mark_bot_angle)), int(mark_bot_pos.y+DISTANCE_ORIENTATION*sin(mark_bot_angle)))

        DISTANCE_ORIENTATION=DISTANCE_FACTOR*mark_bot_pos.dist(ballPos)
        if(DISTANCE_ORIENTATION<200):
            DISTANCE_ORIENTATION=200
        x=int(mark_bot_pos.x+ DISTANCE_ORIENTATION*cos(best_angle))
        y=int(mark_bot_pos.y+ DISTANCE_ORIENTATION*sin(best_angle))
        
        #print "marking bot "+str(self.bot_to_mark)+" using "+str(self.bot_id)
        self.sParam.GoToPointP.x=x
        self.sParam.GoToPointP.y=y
        self.sParam.GoToPointP.finalslope = ballPos.normalizeAngle(best_angle+pi)

        sGoToPoint.execute(self.sParam, state, self.bot_id, pub) 

        # self.sParam.GoToPointP.x=state.ballPos.x
        # self.sParam.GoToPointP.y=state.ballPos.y
        # self.sParam.GoToPointP.max_omega=MAX_BOT_OMEGA/2;

        # sTurnToPoint.execute(self.sParam, state, self.bot_id, pub)       
    def isComplete(self, state):
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        # No parameter to update here
        pass    