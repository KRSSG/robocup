from tactic import Tactic
import time
import sys

sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
import obstacle
import sGoToPoint
import sStop
from tactic import Tactic
from geometry import Vector2D
import skills_union
import sKick
import sKickToPoint
import sGoToPoint
from math import *
from numpy import inf
from config import *
MAX_DRIBBLE_RANGE = 3
KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
THRES = 0.8
THETA_THRESH = 0.005
GOALIE_MAXX = 2450
GOALIE_MAXY =650

SHIFT = (HALF_FIELD_MAXX/7.0)
DISTANCE_ORIENTATION = 2.5*BOT_RADIUS
e = exp(1)

class TPressureCooker(Tactic):
    def __init__(self,bot_id,state,params=None):
        super(TPressureCooker, self).__init__(bot_id, state, params)
        self.bot_id = bot_id

        self.ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        self.botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))

        self.sParams = skills_union.SParam()


    def threat_with_ball(self, state):
        threat = -1
        away_in_our_side_with_ball = []
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        for away_botID in xrange(len(state.awayPos)):
            if state.awayPos[away_botID].x < 0 and ballPos.dist(state.awayPos[away_botID]) <= DRIBBLER_BALL_THRESH :
                threat = away_botID

        if threat is -1:
            min_dist = inf
            
            for away_bot in xrange(len(state.awayPos)):
                dist = ballPos.dist(Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y)))
                if dist < min_dist:
                    dist =min_dist
                    threat = away_bot

        return threat



    def execute(self, state , pub, opp_bot):

        p_threat = opp_bot
        p_threat_pos = Vector2D(int(state.awayPos[p_threat].x), int(state.awayPos[p_threat].y))



        
        x = p_threat_pos.x+ DISTANCE_ORIENTATION*cos(state.awayPos[p_threat].theta)
        y = p_threat_pos.y+ DISTANCE_ORIENTATION*sin(state.awayPos[p_threat].theta)

        self.sParams.GoToPointP.x = x
        self.sParams.GoToPointP.y = y
        self.sParams.GoToPointP.finalslope = self.botPos.normalizeAngle(state.awayPos[p_threat].theta+pi) 
        sGoToPoint.execute(self.sParams, state, self.bot_id, pub)




    def isComplete(self, state):
        return False

    def updateParams(self, state):
        pass
