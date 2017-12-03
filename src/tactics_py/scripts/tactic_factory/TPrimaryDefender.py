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
R   = DBOX_HEIGHT
SHIFT = HALF_FIELD_MAXX*2/3
BOT_GAP_THETA = 2*asin(BOT_RADIUS*1.0/ R)


DIST_THRES = 286
class TPrimaryDefender(Tactic):
    def __init__(self,bot_id,state,params=None):
        super(TPrimaryDefender, self).__init__( bot_id, state, params)
        self.bot_id      =    bot_id
        #self.bot_id      = 1
        #ballPos    =    Vector2D(state.ballPos.x, state.ballPos.y)
        #self.botPos     =    Vector2D(state.homePos[bot_id].x, state.homePos[bot_id].y)
        self.sParams    =    skills_union.SParam()
        #self.tParams    =    tactics_union.TParam()
        self.iState     =    "POSITION"
        
        #print("chuuuu",self.wall_bots)


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
    def inter_circle_and_line(self, P, m):
        c = m*(-HALF_FIELD_MAXX-P.x) + P.y
        delta = (2*m*c)**2 - 4*(c**2-R**2)*(1+m**2)
        #print("delta",delta)
        if delta < 0:
            return False, C
 
        X = 0.5*(-2*m + sqrt(delta))/ (1+ m**2)
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
    # def find_relative_point(self, sol, idd):
        
    #     A = array([(sol.x-C.x, sol.y-C.y), (sol.y-C.y, C.x-sol.x)])
    #     if not idd :
    #         theta_t = -BOT_GAP_THETA
    #     else:
    #         theta_t = BOT_GAP_THETA
    #     #print("iddddd ",idd,theta_t)   
    #     b = array([cos(theta_t)*R*R, sin(theta_t)*R*R])
    #     X = linalg.solve(A,b)
    #     X[0] += C.x
    #     X[1] += C.y
    #     #print("iddddd ",X)
    #     return Vector2D(int(X[0]), int(X[1]))

    def ball_in_our_possession(self,state):
        ballPos      = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        for our_bots in xrange(len(state.homePos)):
            botPos  = Vector2D(int(state.homePos[our_bots].x), int(state.homePos[our_bots].y))
            if ballPos.dist(botPos) <= DRIBBLER_BALL_THRESH :
                return True

        return False


    def execute(self, state , pub):
        #bot_list = [self.bot_id, self.bot_id]
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

        
        if ballPos.dist(botPos) <= DRIBBLER_BALL_THRESH :
            self.sParams.KickToPointP.x = 0
            self.sParams.KickToPointP.y = 0
            self.sParams.KickToPointP.power = 7
            sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
            return

        if state.ballPos.x < HALF_FIELD_MAXX and fabs(state.ballPos.y) < HALF_FIELD_MAXY:

            threat_bot, min_dist =  self.threat_with_ball(state)
            ball_bool, angle = self.ball_velocity_direction(state)
            #print("min dist ,bot ",min_dist,threat_bot)
            threat_pointing_goal = False
            if min_dist < BOT_BALL_DIST_THRESH :
                
                threat_bot_pos = Vector2D(int(state.awayPos[threat_bot].x), int(state.awayPos[threat_bot].y)) 
                threat_bot_theta = state.awayPos[threat_bot].theta
                bool_sol, sol = self.inter_circle_and_line(threat_bot_pos,tan(threat_bot_theta))
                #print("threat1",sol.x, sol.y,bool_sol)
                

                if not bool_sol:
                    threat_bot_theta = threat_bot_pos.angle(C)
                    bool_sol, sol = self.inter_circle_and_line(threat_bot_pos,tan(threat_bot_theta) ) 
                else:
                    threat_pointing_goal = True
            else:
                
                p_threat_id  = self.primary_threat(state, threat_bot)
                threat_bot_pos = Vector2D(int(state.awayPos[p_threat_id].x), int(state.awayPos[p_threat_id].y))
                threat_bot_theta = state.awayPos[p_threat_id].theta
                
                bool_sol, sol = self.inter_circle_and_line(threat_bot_pos, tan(threat_bot_theta))
                #print("pthreat1",p_threat_id,bool_sol)
                if not bool_sol:
                    threat_bot_theta = threat_bot_pos.angle(C)
                    bool_sol, sol = self.inter_circle_and_line(threat_bot_pos, tan(threat_bot_theta))
                else:
                    threat_pointing_goal = True

            #print(threat_pointing_goal,threat_bot_theta)
            self.sParams.GoToPointP.x =  sol.x
            self.sParams.GoToPointP.y = -1*sol.y*0.7

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
             
        if self.ball_in_our_possession(state):
            return
        else:
            if ballPos.dist(botPos) < DBOX_WIDTH:       
                if state.ballPos.x < -HALF_FIELD_MAXX*0.5 and state.ballPos.x > -HALF_FIELD_MAXX + DBOX_WIDTH*1.1:
                    #botPos_y = tan(angle) * (-HALF_FIELD_MAXX + DBOX_WIDTH*0.8 - state.ballPos.x) +state.ballPos.y
                    #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                    #print ("Bot placed at ......................................",-HALF_FIELD_MAXX*19.5/20, botPos_y)
                    self.sParams.GoToPointP.x = state.ballPos.x
                    self.sParams.GoToPointP.y = state.ballPos.y
                    self.sParams.GoToPointP.finalVelocity = 0
                    self.sParams.GoToPointP.finalslope = ballPos.normalizeAngle(angle+pi) 
                    sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
            #print("align with ball velocity ")

        # try:
        #     theta_pri_ball =  atan(state.ballVel.y*1.0/ state.ballVel.x)
        # except:
        #     try:
        #         theta_pri_ball = pi*0.5*fabs(state.ballVel.y)/ state.ballVel.y
        #     except:
        #         theta_pri_ball = 0
        ##############################################################################################################3
        # if (botPos.x <= -HALF_FIELD_MAXX+DBOX_WIDTH*1.1 and fabs(botPos.y) <= DBOX_HEIGHT) or botPos.x > -1*SHIFT:
        #     self.sParams.GoToPointP.x = -HALF_FIELD_MAXX+DBOX_WIDTH*1.1
        #     if botPos.y is 0:
        #         self.sParams.GoToPointP.y = 0
        #     else:
        #         self.sParams.GoToPointP.y = DBOX_HEIGHT*0.6*fabs(botPos.y)/botPos.y
        #     self.sParams.GoToPointP.finalslope = ballPos.angle(botPos)
        #     #print("vsdvsdvgsdgvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv")
        #     sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
        ##########################################################################################################################   
        #elif (botPos.x <= -HALF_FIELD_MAXX+DBOX_WIDTH*0.8 and fabs(botPos.y - HALF_FIELD_MAXY) <= DBOX_HEIGHT) or botPos.x > -1*SHIFT:


        # if ballPos.dist(botPos) < DRIBBLER_BALL_THRESH*1.3 :
        #         self.sParams.KickToPointP.x = 0
        #         self.sParams.KickToPointP.y = 0
        #         self.sParams.KickToPointP.power  = 7
        #         self.sParams.KickP.power = 7
        #         sKickToPoint.execute(self.sParams, state, self.bot_id, pub)




        # if state.ballPos.x >  -HALF_FIELD_MAXX/3:
        #     ##print(fabs(state.ballPos.y) , HALF_FIELD_MAXY*8/9)
        #     if fabs(state.ballPos.y) < HALF_FIELD_MAXY*6/9: 
        #         self.sParams.GoToPointP.x = botPos.x
        #         self.sParams.GoToPointP.y = botPos.y
        #         self.sParams.GoToPointP.finalVelocity = 0
        #         self.sParams.GoToPointP.finalslope = ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.bot_id].y) / (state.ballPos.x - state.homePos[self.bot_id].x))) 

        #         #print("Adjusting.................",-1*(OUR_GOAL_MAXY*state.ballPos.y/GOALIE_MAXY),state.ballPos.y)
        #         sGoToPoint.execute(self.sParams, state, self.bot_id, pub)

        
        # if state.ballPos.x < 0 and state.ballPos.x > -HALF_FIELD_MAXX + DBOX_WIDTH:
            ##print(int(state.ballPos.x))
        


            # if ballPos.dist(state.homePos[self.bot_id]) < DRIBBLER_BALL_THRESH :
            #     self.sParams.KickToPointP.x = 0
            #     self.sParams.KickToPointP.y = 0
            #     self.sParams.KickToPointP.power     = 7
            #     self.sParams.KickP.power = 7
            #     #sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
                
            #     #print("kick")
            #     sKickToPoint.execute(self.sParams, state , self.bot_id,pub)


        


        # if p_threat_id is not -1:
        #     p_threat_pos = Vector2D(int(state.awayPos[p_threat_id].x), int(state.awayPos[p_threat_id].y)) 
        #     sol_left     = self.inter_circle_and_line(p_threat, upper_limit, C, R, sol_left)
        #     sol_mid      = self.inter_circle_and_line(p_threat, C, C, R, sol_mid)
        #     sol_right    = self.inter_circle_and_line(p_threat, lower_limit, C, R, sol_right)

            

        #     if upper_dist > lower_dist + DIST_THRES :
        #         self.sParams.GoToPointP.x = 0.5*(sol_mid.x + sol_right.x)
        #         self.sParams.GoToPointP.y = 0.5*(sol_mid.y + sol_right.y)
        #         sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
        #     if upper_dist  + DIST_THRES < lower_dist :
        #         self.sParams.GoToPointP.x = 0.5*(sol_left.x + sol_mid.x)
        #         self.sParams.GoToPointP.y = 0.5*(sol_left.y + sol_mid.y)
        #         sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
        #     if fabs(upper_dist - lower_dist) < DIST_THRES :
        #         self.sParams.GoToPointP.x = 0.5*(sol_mid.x + sol_mid.x)
        #         self.sParams.GoToPointP.y = 0.5*(sol_mid.y + sol_mid.y)
        #         sGoToPoint.execute(self.sParams, state, self.bot_id, pub)



        #     if ballPos.dist(botPos) < DRIBBLER_BALL_THRESH*1.3 :
        #         self.sParams.KickToPointP.x = 0
        #         self.sParams.KickToPointP.y = 0
        #         self.sParams.KickToPointP.power  = 7
        #         self.sParams.KickP.power = 7
        #         sKickToPoint.execute(self.sParams, state, self.bot_id, pub)

        