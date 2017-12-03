

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
from numpy import inf,array,linalg
from config import *
MAX_DRIBBLE_RANGE = 3
KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
THRES = 0.8
THETA_THRESH = 0.005
GOALIE_MAXX = 2450
GOALIE_MAXY =650
BOT_BALL_DIST_THRESH = HALF_FIELD_MAXX/ 7

C   = Vector2D(int(-HALF_FIELD_MAXX), 0)
R   = DBOX_HEIGHT/1.5
SHIFT = HALF_FIELD_MAXX*2/3
BOT_GAP_THETA = 2*asin(BOT_RADIUS*1.0/ R)


DIST_THRES = 286
class TGoalie(Tactic):
    def __init__(self,bot_id,state,params=None):
        super(TGoalie, self).__init__(bot_id, state, params)
        self.bot_id = bot_id
        self.bot_id = bot_id
        self.ballAim = Vector2D()
        self.goalieTarget = Vector2D(0,0)
        self.ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        self.botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        self.ballVel = Vector2D(int(state.ballVel.x) , int(state.ballVel.y))
        self.sParams = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MINY)

        self.GOAL_UPPER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MAXY*3)
        self.GOAL_LOWER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY*3)
        
        self.sParams = skills_union.SParam()
        

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
    def inter_circle_and_line(self, P, m, C, R):
        c = m*(-HALF_FIELD_MAXX-P.x) + P.y
        delta = (2*m*c)**2 - 4*(c**2-R**2)*(1+m**2)
        #print("delta",delta)
        if delta < 0:
            return False, C
 
        X = 0.5*(-2*m + sqrt(delta))*1.0/ (1+ m**2)
        Y = m*X + c
        #print("xxx",X)
        x = X - HALF_FIELD_MAXX
        #print("XxXxX",x)
        y = Y
        #print("why",y)
        P = Vector2D(int(x), int(y)) 

        return True, P
    #this function returns true if the direction of ball's velocity vector lies within the open angle of the goal
    def ball_velocity_direction(self,state):
        ballVel = Vector2D(int(state.ballVel.x), int(state.ballVel.y))
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        Upper_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MAXY / 2 + BOT_BALL_THRESH))
        Lower_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MINY / 2 - BOT_BALL_THRESH))

        alpha_1 = atan((Upper_limit.y - ballPos.y)/ (ballPos.x - Upper_limit.x)) 
        alpha_2 = atan((Lower_limit.y - ballPos.y)/ (ballPos.x - Lower_limit.x))

        if ballVel.x < 0 or 1:
            try:
                ball_vel_angle = atan(state.ballVel.y*-1 / state.ballVel.x)
            except:
                if state.ballVel.y < 0:
                    ball_vel_angle = pi/2
                elif state.ballVel.y >0:
                    ball_vel_angle = pi*-0.5
                else:
                    ball_vel_angle = 0
        else:
            return False,inf
        #deciding the direction of the ball's velocity
        if ballVel.y > 0:
            if(ball_vel_angle > alpha_2 and ball_vel_angle < alpha_1):
                return True,ball_vel_angle
            else :
                return False,ball_vel_angle
        else:
            if(abs(ball_vel_angle) > abs(alpha_1) and abs(ball_vel_angle) < abs(alpha_2)):
                return True,ball_vel_angle
            else:
                return False,ball_vel_angle

    def threat_with_ball(self,state):
        threat = -1
        away_in_our_side_with_ball = []
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        for away_bot_id in xrange(len(state.awayPos)):
            if state.awayPos[away_bot_id].x < 0 and ballPos.dist(state.awayPos[away_bot_id]) <= DRIBBLER_BALL_THRESH :
                threat = away_bot_id
        min_dist = inf
        if threat is -1:
            
            ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
            for away_bot in xrange(len(state.awayPos)):
                dist = ballPos.dist(Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y)))
                if dist < min_dist:
                    min_dist = dist
                    threat = away_bot


        #if min_dist < BOT_BALL_DIST_THRESH:
        return threat,min_dist
        #else:
        #    return -1
            



    #returns the bot_id of the primary threat i.e, the bot which is likely to recieve the pass else if the ball's velocity is less than certain threshold function returns -1
    def primary_threat(self,state, threat_bot):
        thresh = 100000.0
        threat = -1
        away_in_our_side = []
        dist_list =[]
        angle_diff_list = []
        angle_diff_list_ball = []

        #calculate bot_id of opponent on our goalie side
        #threat_bot = threat_with_ball(state)
        ball_dir_in_goal, ball_vel_angle = self.ball_velocity_direction(state)
        #print("threat" ,threat_bot)
        #print("ball vle angle", ball_vel_angle)
        if threat_bot is not -1:
            threatPos = Vector2D(int(state.awayPos[threat_bot].x), int(state.awayPos[threat_bot].y))

            for away_bot in  xrange(len(state.awayPos)):
                #print("chutiya",threat_bot,away_bot)
                if away_bot is threat_bot:
                    continue
                #print("chutiya2",threat_bot,away_bot)
                angle_diff = fabs(state.awayPos[threat_bot].theta - state.awayPos[away_bot].theta)

                angle_diff_list.append([away_bot, exp(-1.0*angle_diff/ pi)])
                
                dist = threatPos.dist(Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y)))
                dist_list.append([away_bot, exp(-1*dist/ HALF_FIELD_MAXY)])
                if ball_vel_angle:
                    angle_diff_ball = fabs(ball_vel_angle - state.awayPos[away_bot].theta)
                else:
                    angle_diff_ball = inf
                angle_diff_list_ball.append([away_bot, exp(-1.0*angle_diff_ball/ pi)])
                #exp(-1.0*angle_diff_ball/ pi)


            scores = []
            weights = [0.6, 0.9, 0.8]
            # print("angle ",angle_diff_list)
            #print(threat_bot,"grggrgtr")
            for i in xrange(len(state.awayPos)-1):
                sc = angle_diff_list[i][1]*weights[0] + dist_list[i][1]*weights[1] + angle_diff_list_ball[i][1]*weights[2]
                scores.append([angle_diff_list[i][0], sc])
            scores.sort(key = lambda x:x[1], reverse=True)
            #print("pThreat ",scores)
            return scores[0][0]
        else:
            return -1
    #returns secondary_threat_id if none is found returns -1
    def secondary_threat(self,state,vec,primary_threat_bot = -1):
        away_in_our_side = []
        home_in_our_side = []

        Upper_limit = Vector2D(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.2 + BOT_BALL_THRESH)
        Lower_limit = Vector2D(-HALF_FIELD_MAXX, OUR_GOAL_MINY / 1.2 - BOT_BALL_THRESH)



        #our bots on our side
        for home_bot_id in xrange(len(state.homeDetected)):
            if state.homePos[home_bot_id].x < 0 :
                home_in_our_side.append(home_bot_id)

        #opponents closer to our goalie side not the primary_threat
        for away_bot_id in xrange(len(state.awayDetected)):
            if state.awayPos[away_bot_id].x < 0 and away_bot_id is not primary_threat_bot:
                away_in_our_side.append(away_bot_id)

        #calculate max open angle
        Max_Open_Angle = 0.0
        secondary_threat_id = -1
        for away_bot_id in away_in_our_side:
            OpenAngle = abs(self.angle_at_vextex(state.awayPos[away_bot_id],Upper_limit,Lower_limit))
            for bots in away_in_our_side:
                if bots is not away_bot_id:
                    if self.point_in_traingle(state.awayPos[away_bot_id], Upper_limit, Lower_limit, state.awayPos[bots]):
                        temp_theta = 2*asin(BOT_RADIUS/((state.awayPos[away_bot_id]-state.awayPos[bots]).absSq()))
                        OpenAngle -= temp_theta
            for bots in home_in_our_side:
                if self.point_in_traingle(state.awayPos[away_bot_id],Upper_limit,Lower_limit,state.homePos[bots]):
                    temp_theta = 2*asin(BOT_RADIUS/((state.awayPos[away_bot_id]-state.homePos[bots]).absSq()))
                    OpenAngle -= temp_theta
            if OpenAngle > Max_Open_Angle :
                 Max_Open_Angle = OpenAngle
                 secondary_threat_id = away_bot_id
        #return secondary_threat_id
        return secondary_threat_id




    def execute(self, state , pub):
        #bot_list = [self.bot_id, self.bot_id]

        if self.ballPos.dist(self.botPos) <= DRIBBLER_BALL_THRESH:
            self.sParams.KickToPointP.x = 0
            self.sParams.KickToPointP.y = 0
            self.sParams.KickToPointP.power = 7
            sKickToPoint.execute(self.sParams, state, self.bot_id, pub)

            print("___________KICK______________")
            return
        #self.wall_bots = wall


        #print("fuck off bitches...........................................................................................................................")
        upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MAXY ))
        lower_limit = Vector2D(int(upper_limit.x), int(-1 * upper_limit.y))
        # p_def       = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        # 

        
        botPos       = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        ballPos      = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        goalieBot  = Vector2D(int(state.homePos[4].x), int(state.homePos[4].y))
        upper_dist = goalieBot.dist(upper_limit)
        lower_dist = goalieBot.dist(lower_limit)
        
        sol = Vector2D(int(-HALF_FIELD_MAXX + DBOX_WIDTH*1.1), 0)
        ball_bool, angle = self.ball_velocity_direction(state)



        if state.ballPos.x < HALF_FIELD_MAXX and fabs(state.ballPos.y) < HALF_FIELD_MAXY:

            threat_bot, min_dist =  self.threat_with_ball(state)
            ball_bool, angle = self.ball_velocity_direction(state)
            #print("min dist ,bot ",min_dist,threat_bot)
            threat_pointing_goal = False
            if min_dist < BOT_BALL_DIST_THRESH :
                
                threat_bot_pos = Vector2D(int(state.awayPos[threat_bot].x), int(state.awayPos[threat_bot].y)) 
                threat_bot_theta = state.awayPos[threat_bot].theta

                bool_sol, sol = self.inter_circle_and_line(threat_bot_pos,tan(threat_bot_theta) , C, R)
                #print("threat1",threat_bot,bool_sol)
                if not bool_sol:
                    threat_bot_theta = threat_bot_pos.angle(C)
                    bool_sol, sol = self.inter_circle_and_line(threat_bot_pos,threat_bot_theta , C, R) 
                else:
                    threat_pointing_goal = True
            else:
                
                p_threat_id  = self.primary_threat(state, threat_bot)
                threat_bot_pos = Vector2D(int(state.awayPos[p_threat_id].x), int(state.awayPos[p_threat_id].y))
                threat_bot_theta = state.awayPos[p_threat_id].theta
                
                bool_sol, sol = self.inter_circle_and_line(threat_bot_pos, threat_bot_theta, C, R)
                #print("pthreat1",p_threat_id,bool_sol)
                if not bool_sol:
                    threat_bot_theta = threat_bot_pos.angle(C)
                    bool_sol, sol = self.inter_circle_and_line(threat_bot_pos, tan(threat_bot_theta), C, R)
                else:
                    threat_pointing_goal = True

            #print(threat_pointing_goal,threat_bot_theta)
            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*29/30
            self.sParams.GoToPointP.y = 1.0*sol.y*0.7

            if threat_pointing_goal:
                if not (threat_bot_theta > pi/2 or threat_bot_theta < -pi/2 ):
                    threat_pointing_goal = False

            #print(threat_pointing_goal)
            if threat_pointing_goal:
                self.sParams.GoToPointP.finalslope = threat_bot_pos.normalizeAngle(threat_bot_theta+pi)
            else:
                self.sParams.GoToPointP.finalslope = threat_bot_pos.normalizeAngle(threat_bot_theta)

            #print("bot theta ",180*threat_bot_theta/pi, 180*threat_bot_pos.normalizeAngle(threat_bot_theta)/ pi)
            sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
             

        if ballPos.dist(botPos) < DBOX_WIDTH :    
            if state.ballPos.x < -HALF_FIELD_MAXX + DBOX_WIDTH and fabs(state.ballPos.y) < DBOX_HEIGHT:  
                self.sParams.GoToPointP.x = state.ballPos.x
                self.sParams.GoToPointP.y = state.ballPos.y
                self.sParams.GoToPointP.finalVelocity = 0

                if state.ballVel.x:
                    theta_ball = ballPos.normalizeAngle(atan(state.ballVel.y*1.0/ state.ballVel.x))
                else:
                    theta_ball = ballPos.angle(botPos)

                self.sParams.GoToPointP.finalslope = theta_ball 
                sGoToPoint.execute(self.sParams, state, self.bot_id, pub)

    def isComplete(self, state):
        return False

    def updateParams(self, state):
        pass

        