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
e = exp(1)
from enum import Enum


DIST_THRES = 286
class TPosition(Tactic):
    def __init__(self,botID,state,params=None):

        self.botID      =    botID
        self.botID      = 1
        #self.ballPos    =    Vector2D(state.ballPos.x, state.ballPos.y)
        #self.botPos     =    Vector2D(state.homePos[botID].x, state.homePos[botID].y)
        self.sParams    =    skills_union.SParam()
        #self.tParams    =    tactics_union.TParam()
        self.iState     =    "POSITION"


    def  isComplete(self,state,tParams):
        return iState == "KICK"

    def isActiveTactic():
        return iState is not "KICK"

    def updateParams(self, state):
        pass
    #defines triangle properties
    
        #returns the angle at first vertex
    def angle_at_vextex(P1,P2,P3):
        a     = P2.dist(P3)
        b     = P1.dist(P3)
        c     = P2.dist(P1)
        theta = acos((b**2 + c**2 - a**2)/(2*b*c))
        return theta

    #returns whether a pt lies inside a triangle
    def point_in_traingle(P1, P2, P3, P):
        def area(p1,p2,p3):
            return abs(p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y))/2.0

        a  = area(P1,P2,P3)
        a1 = area(P,P2,P3)
        a2 = area(P1,P,P3)
        a3 = area(P1,P2,P)

        return a == a1+a2+a3


    #returns the intersection point of a line and circle
    def inter_circle_and_line(self, P1, P2, C, R, P):
        #here P1 always represents the threat position
        # shifht origin to center of the  circle
        tx1 = P1.x - C.x
        tx2 = P2.x - C.x
        ty1 = P1.y - C.y
        ty2 = P2.y - C.y


        #calculate the intersection of the line and circle of radius R
        dx  = tx2 - tx1
        dy  = ty2 - ty1
        dr  = sqrt(pow(dx, 2) + pow(dy, 2))
        D  = tx1 * ty2 - tx2 * ty1
        P.x = (D * dy + sign(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2))
        P.y = ((-D * dx) + abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2))
        if P.x < 0:
            P.x = (D * dy - sign(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2))

        if P1.y > 0:
            if P.y < 0:
                P.y = (-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2))
            elif P.y > 0:
                P.y = (-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2))

        #convert the co-ordinates back to the original system
        P.x = P.x + C.x
        P.y = P.y + C.y
        return P
    #this function returns true if the direction of ball's velocity vector lies within the open angle of the goal
    def ball_velocity_direction(self,state):
        ballVel = Vector2D(int(state.ballVel.x), int(state.ballVel.y))
        ball_pos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        Upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MAXY / 1.2 + BOT_BALL_THRESH))
        Lower_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MINY / 1.2 - BOT_BALL_THRESH))
        try:
            alpha_1 = atan((Upper_limit.y - ball_pos.y)/ (ball_pos.x - Upper_limit.x))
        except:
            alpha_1 = pi*0.5*fabs((Upper_limit.y - ball_pos.y))/(Upper_limit.y - ball_pos.y)

        try:
            alpha_2 = atan((Lower_limit.y - ball_pos.y)/ (ball_pos.x - Lower_limit.x))
        except:
            alpha_2 = pi*0.5*fabs(Lower_limit.y - ball_pos.y)/ (Lower_limit.y - ball_pos.y)


        if ballVel.x < 0:
            ball_vel_angle =atan(state.ballVel.y*-1 / state.ballVel.x)
        else:
            return False
        #deciding the direction of the ball's velocity
        if ballVel.y > 0:
            if(ball_vel_angle > alpha_2 and ball_vel_angle < alpha_1):
                return True
            else :
                return False
        else:
            if(abs(ball_vel_angle) > abs(alpha_1) and abs(ball_vel_angle) < abs(alpha_2)):
                return True
            else:
                return False

    def threat_with_ball(self,state):
        threat = -1
        away_in_our_side_with_ball = []
        ballPos = Vector2D(state.ballPos.x, state.ballPos.y)
        for away_botID in xrange(len(state.awayDetected)):
            if state.awayPos[away_botID].x < 0 and ballPos.dist(state.awayPos[away_botID]) <= BOT_RADIUS*1.05 :
                away_in_our_side_with_ball.append(away_botID)
        



    #returns the botID of the primary threat i.e, the bot which is likely to recieve the pass else if the ball's velocity is less than certain threshold function returns -1
    def primary_threat(self,state):
        thresh = 100000.0
        threat = -1
        away_in_our_side = []
        if (sqrt(pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) <= LOW_BALL_VELOCITY_THRES):
            #ball is not getting passed
            return -1
        else:
            #calculate botID of opponent on our goalie side
            if not self.ball_velocity_direction(state):
                for i in xrange(len(state.awayDetected)):
                    if state.awayPos[i].x < 0:
                        away_in_our_side.append(i)
                for itr in away_in_our_side:
                    if state.awayPos[itr].x < thresh :
                        threat = itr
                        thresh = state.awayPos[itr].x
                return threat
            else:
                return -1
    #returns secondary_threat_id if none is found returns -1
    def secondary_threat(self,state,vec,primary_threat_bot = -1):
        away_in_our_side = []
        home_in_our_side = []

        Upper_limit = Vector2D(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.2 + BOT_BALL_THRESH)
        Lower_limit = Vector2D(-HALF_FIELD_MAXX, OUR_GOAL_MINY / 1.2 - BOT_BALL_THRESH)



        #our bots on our side
        for home_botID in xrange(len(state.homeDetected)):
            if state.homePos[home_botID].x < 0 :
                home_in_our_side.append(home_botID)

        #opponents closer to our goalie side not the primary_threat
        for away_botID in xrange(len(state.awayDetected)):
            if state.awayPos[away_botID].x < 0 and away_botID is not primary_threat_bot:
                away_in_our_side.append(away_botID)

        #calculate max open angle
        Max_Open_Angle = 0.0
        secondary_threat_id = -1
        for away_botID in away_in_our_side:
            OpenAngle = abs(self.angle_at_vextex(state.awayPos[away_botID],Upper_limit,Lower_limit))
            for bots in away_in_our_side:
                if bots is not away_botID:
                    if self.point_in_traingle(state.awayPos[away_botID], Upper_limit, Lower_limit, state.awayPos[bots]):
                        temp_theta = 2*acos(BOT_RADIUS/((state.awayPos[away_botID]-state.awayPos[bots]).absSq()))
                        OpenAngle -= temp_theta
            for bots in home_in_our_side:
                if self.point_in_traingle(state.awayPos[away_botID],Upper_limit,Lower_limit,state.homePos[bots]):
                    temp_theta = 2*acos(BOT_RADIUS/((state.awayPos[away_botID]-state.homePos[bots]).absSq()))
                    OpenAngle -= temp_theta
            if OpenAngle > Max_Open_Angle :
                 Max_Open_Angle = OpenAngle
                 secondary_threat_id = away_botID
        #return secondary_threat_id
        return secondary_threat_id

    def execute(self, state , pub):
        C           = Vector2D(int(-HALF_FIELD_MAXX), 0)
        upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MAXY / 1.30))
        lower_limit = Vector2D(int(upper_limit.x), int(-1 * upper_limit.y))
        p_def       = Vector2D(int(state.homePos[self.botID].x), int(state.homePos[self.botID].y))
        R           = HALF_FIELD_MAXX /2.5

        
        botPos       = Vector2D(int(state.homePos[self.botID].x), int(state.homePos[self.botID].y))
        ballPos      = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        goalieBot  = Vector2D(int(state.homePos[4].x), int(state.homePos[4].y))
        upper_dist = goalieBot.dist(upper_limit)
        lower_dist = goalieBot.dist(lower_limit)

        if ballPos.dist(botPos) < DRIBBLER_BALL_THRESH*1.3 :
                self.sParams.KickToPointP.x = 0
                self.sParams.KickToPointP.y = 0
                self.sParams.KickToPointP.power  = 7
                self.sParams.KickP.power = 7
                sKickToPoint.execute(self.sParams, state, self.botID, pub)

        p_threat_id  = self.primary_threat(state)


        if p_threat_id is not -1:
            p_threat_pos = Vector2d(int(state.awayPos[p_threat_id].x), int(state.awayPos[p_threat_id].y)) 
            sol_left     = self.inter_circle_and_line(p_threat, upper_limit, C, R, sol_left)
            sol_mid      = self.inter_circle_and_line(p_threat, C, C, R, sol_mid)
            sol_right    = self.inter_circle_and_line(p_threat, lower_limit, C, R, sol_right)

            

            if upper_dist > lower_dist + DIST_THRES :
                self.sParams.GoToPointP.x = 0.5*(sol_mid.x + sol_right.x)
                self.sParams.GoToPointP.y = 0.5*(sol_mid.y + sol_right.y)
                sGoToPoint.execute(self.sParams, state, self.botID, pub)
            if upper_dist  + DIST_THRES < lower_dist :
                self.sParams.GoToPointP.x = 0.5*(sol_left.x + sol_mid.x)
                self.sParams.GoToPointP.y = 0.5*(sol_left.y + sol_mid.y)
                sGoToPoint.execute(self.sParams, state, self.botID, pub)
            if fabs(upper_dist - lower_dist) < DIST_THRES :
                self.sParams.GoToPointP.x = 0.5*(sol_mid.x + sol_mid.x)
                self.sParams.GoToPointP.y = 0.5*(sol_mid.y + sol_mid.y)
                sGoToPoint.execute(self.sParams, state, self.botID, pub)



            if self.ballPos.dist(botPos) < DRIBBLER_BALL_THRESH*1.3 :
                self.sParams.KickToPointP.x = 0
                self.sParams.KickToPointP.y = 0
                self.sParams.KickToPointP.power  = 7
                self.sParams.KickP.power = 7
                sKickToPoint.execute(self.sParams, state, self.botID, pub)

        
