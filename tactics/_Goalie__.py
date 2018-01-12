from tactic import Tactic
import time
import sys


from geometry import *
import skills_union
from config import *
from tactic import Tactic
from geometry import Vector2D
import skills_union
import enum
import sKickToPoint
import sGoalie
from math import *
from numpy import inf,array,linalg
from config import *
from utils import math_functions

class TGoalie(Tactic):
    def __init__(self,bot_id,state,params=None):
        super(TGoalie, self).__init__(bot_id, state, params)
        self.bot_id = bot_id
        self.ballAim = Vector2D()
        self.goalieTarget = Vector2D(0,0)
        self.sParams = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MINY)
        self.ballPrevX_Velocity = 0
        self.GOAL_UPPER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MAXY*3)
        self.GOAL_LOWER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY*3)

        self.sParams = skills_union.SParam()

    class State(enum.Enum):
        # Opponent has the ball and we need to block
        block = 1
        # The ball is moving towards our goal and we should catch it.
        intercept = 2
        # Get the ball out of our defense area.
        clear = 3
        # Ball in our possession
        chill = 4

        # TO DO Add more states for refree plays


    def getState(self,state):
        attacker_id = state.opp_bot_closest_to_ball
        attacker_pos = Vector2D (int(state.awayPos[attacker_id].x),int(state.awayPos[attacker_id].y))
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        attacker_dist = ballPos.dist(attacker_pos)
        self.ballPrevX_Velocity = (self.ballPrevX_Velocity + state.ballVel.x)/2
        if False :  # check for ref_play
            pass
        elif self.ball_in_our_dbox(state) :
            return TGoalie.State.clear
        elif state.ball_in_our_possession :
            return TGoalie.State.chill
        elif int(state.ballVel.x) < -1 and int(self.ballPrevX_Velocity) < -7:  # TO DO SIGNS ACCORDING TO FIELD POSITION
            return TGoalie.State.intercept
        elif attacker_dist < DRIBBLER_BALL_THRESH and (state.homePos[self.bot_id].x-state.ballPos.x)*(state.ballPos.x-state.awayPos[state.opp_bot_closest_to_ball].x) > 0 :
            return TGoalie.State.block
        else :
            return TGoalie.State.chill


    def execute(self, state , pub):
        #bot_list = [self.bot_id, self.bot_id]
        print "BALL_POS : ",state.ballPos.x,",",state.ballPos.y
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        ballVel = Vector2D(int(state.ballVel.x) , int(state.ballVel.y))
        distance = botPos.dist(ballPos)
        attacker_id = state.opp_bot_closest_to_ball
        attacker_pos = Vector2D (int(state.awayPos[attacker_id].x),int(state.awayPos[attacker_id].y))
        gameState = self.getState(state)

        if gameState == TGoalie.State.block:
            print ("ATTACKER_HAS_THE_BALL")
            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX+2*BOT_RADIUS
            if (ballPos.x-attacker_pos.x) != 0 :
                y = ballPos.y + (ballPos.y-attacker_pos.y)*abs((-HALF_FIELD_MAXX+2*BOT_RADIUS-ballPos.x)/(ballPos.x-attacker_pos.x))
            else :
                y = ballPos.y
            y = min(y,OUR_GOAL_MAXY - BOT_RADIUS)
            y = max(y,OUR_GOAL_MINY + BOT_RADIUS)
            self.sParams.GoToPointP.y = y
            finalPos = Vector2D(self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
            self.sParams.GoToPointP.finalSlope = ballPos.angle(finalPos)
            sGoalie.execute(self.sParams, state, self.bot_id, pub)
        elif gameState == TGoalie.State.intercept:
            print ("___BALL_APPROACHING______")
            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX+2*BOT_RADIUS
            y = ballPos.y + ballVel.y*abs((-HALF_FIELD_MAXX+2*BOT_RADIUS-ballPos.x)/ballVel.x)
            y = min(y,OUR_GOAL_MAXY - BOT_RADIUS)
            y = max(y,OUR_GOAL_MINY + BOT_RADIUS)
            self.sParams.GoToPointP.y = y
            finalPos = Vector2D(self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
            self.sParams.GoToPointP.finalSlope = ballPos.angle(finalPos)
            sGoalie.execute(self.sParams, state, self.bot_id, pub)
        elif gameState == TGoalie.State.chill :
            print ("___BALL_NOT_APPROACHING______")
            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX+2*BOT_RADIUS
            y = ballPos.y
            y = min(y,OUR_GOAL_MAXY - BOT_RADIUS)
            y = max(y,OUR_GOAL_MINY + BOT_RADIUS)
            self.sParams.GoToPointP.y = y
            self.sParams.GoToPointP.finalSlope = ballPos.angle(botPos)
            sGoalie.execute(self.sParams, state, self.bot_id, pub)
        elif gameState == TGoalie.State.clear:
            print (" ___CLEAR_THE_BALL________")
            # TO DO decide the point where the ball is to be kicked
            self.sParams.KickToPointP.x = 0
            self.sParams.KickToPointP.y = 0
            self.sParams.KickToPointP.power = 7
            sKickToPoint.execute(self.sParams,state, self.bot_id,pub) 
        else :
            print ("___REFREE_PLAY__________")
            # TO DO add refree conditions here


    def isComplete(self, state):
        return False

    def updateParams(self, state):
        pass
