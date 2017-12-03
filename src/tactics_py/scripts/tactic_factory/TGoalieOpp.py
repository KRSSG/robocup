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
import sGoToBall
from math import *
from numpy import inf
from config import *
HALF_FIELD_MAXX *=-1
DBOX_WIDTH*=-1
MAX_DRIBBLE_RANGE = 3
KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
THRES = 0.8
THETA_THRESH = 0.005
GOALIE_MAXX = 2450
GOALIE_MAXY =650


SHIFT = (HALF_FIELD_MAXX/7.0)
e = exp(1)

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
        
        #placing the goalie


    def ball_velocity_direction(self,state):
        ballVel = Vector2D(int(state.ballVel.x), int(state.ballVel.y))
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        Upper_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MAXY / 2 + BOT_BALL_THRESH))
        Lower_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MINY / 2 - BOT_BALL_THRESH))

        alpha_1 = atan((Upper_limit.y - ballPos.y)/ (ballPos.x - Upper_limit.x)) 
        alpha_2 = atan((Lower_limit.y - ballPos.y)/ (ballPos.x - Lower_limit.x))

        if ballVel.x < 0:
            ball_vel_angle = atan(state.ballVel.y*-1 / state.ballVel.x)
        else:
            return False,inf
        #deciding the direction of the ball's velocity
        if ballVel.y > 0:
            if(ball_vel_angle > alpha_2 and ball_vel_angle < alpha_1):
                return True,ball_vel_angle
            else :
                return False,inf
        else:
            if(abs(ball_vel_angle) > abs(alpha_1) and abs(ball_vel_angle) < abs(alpha_2)):
                return True,ball_vel_angle
            else:
                return False,inf

    def threat_with_ball(self,state):
        threat = -1
        away_in_our_side_with_ball = []
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        for away_botID in xrange(len(state.awayPos)):
            if state.awayPos[away_botID].x < 0 and ballPos.dist(state.awayPos[away_botID]) <= BOT_RADIUS*1.5 :
                away_in_our_side_with_ball.append(away_botID)
        if len(away_in_our_side_with_ball) > 0:
            return away_in_our_side_with_ball[0]
        else :
            return -1

    def determine_line(Y,X,m,x):
        y = m*(x-X) +Y

    def goal_posible(self,state,bot_id):

        botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))

        #print ("chutioya saala ",botPos.x )
        goal_angle_upper = atan((self.GOAL_UPPER.y-botPos.y)*1.0/ (self.GOAL_UPPER.x-botPos.x))
        goal_angle_lower = atan((self.GOAL_LOWER.y-botPos.y)*1.0/ (self.GOAL_LOWER.x-botPos.x))
        goal_angle_mid   = 0.5*(goal_angle_lower+goal_angle_upper)

        
        k1 = False
        k2 = False
        for away_bot in xrange(len(state.awayPos)):
            
            if state.awayPos[away_bot].x > state.homePos[bot_id].x:
                #print (state.awayPos[away_bot].x, state.homePos[self.bot_id].x, "chutiya.....")
                away_BOT = Vector2D (int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y))
                bot_angle = atan((away_BOT.y - botPos.y)*1.0/ (away_BOT.x - botPos.x))

                #print("bhosdiwala angles ",bot_angle, goal_angle_lower, goal_angle_mid, goal_angle_upper, away_bot)
                if bot_angle > goal_angle_lower and bot_angle < goal_angle_mid:
                    k1=True
                     
                if bot_angle > goal_angle_mid and bot_angle < goal_angle_upper:
                    k2=True
               # print(k1,k2)
                    
        if k1 and k2:
            return [False, -1]
        if k1 :
            return [True, 2]
        if k2 :
            return [True, 1]
        if not (k1 or k2):
            return [True, 0]




    def execute(self, state , pub):
        dist = self.ballPos.dist(self.botPos)
        '''     
        if(sqrt(self.ballVel.absSq()) < 0.02*MAX_BOT_SPEED and  fabs(state.ballPos.y) < OUR_GOAL_MAXY and state.ballPos.x > (HALF_FIELD_MAXX- (2 * DBOX_WIDTH) (HALF_FIELD_MAXX/5.0)):
        '''
        
        #sprint("printing ................",bot_x,bot_y,HALF_FIELD_MAXX*-3.5/5)
        #print(len(state.homeVel),"fddddddddddddddddddddddddd")
        
        if not(fabs(state.homePos[self.bot_id].y) < OUR_GOAL_MAXY and state.homePos[self.bot_id].x <= -HALF_FIELD_MAXX + DBOX_WIDTH*0.8 and state.homePos[self.bot_id].x > -HALF_FIELD_MAXX) :

            #self.sParams.GoToPointP.x = state.homePos[self.bot_id].x
            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX + DBOX_WIDTH*0.3
            self.sParams.GoToPointP.y = 0
            #if state.homePos[self.bot_id].y < 0:
            #    self.sParams.GoToPointP.y = OUR_GOAL_MINY
            #else:
            #    self.sParams.GoToPointP.y = OUR_GOAL_MAXY
            self.sParams.GoToPointP.finalVelocity = 0
            self.sParams.GoToPointP.finalslope    = 0
            #print("beyond y and x.................................",self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
            sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
            #print("............................y and x...")
       

        #re-placing the bot 

        elif not (state.homePos[self.bot_id].x <= -HALF_FIELD_MAXX + DBOX_WIDTH*0.3) :

            self.sParams.GoToPointP.x = -HALF_FIELD_MAXX + DBOX_WIDTH*0.8
            self.sParams.GoToPointP.y = 0
            
            self.sParams.GoToPointP.finalVelocity = 0
            self.sParams.GoToPointP.finalslope = 0
            #print("beyond x...................................",self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
            sGoToPoint.execute(self.sParams, state, self.bot_id, pub)


            #print (".......................x..")

        #elif fabs(state.homePos[self.bot_id].x+GOALIE_MAXX) < 150 and fabs(state.homePos[self.bot_id].y) < 80 and  (state.homeVel[self.bot_id].x >= 0 or state.homeVel[self.bot_id].y >= 0) :
            #print "shubham stop"+"**"*50
            #  sStop.execute(self.sParams, state, self.bot_id, pub)
            
        
        #print(state.ballPos.x)

        ball_bool, angle = self.ball_velocity_direction(state)
        # print("angle",angle)
        if ball_bool:
            botPos_y = tan(angle) * (-HALF_FIELD_MAXX - state.ballPos.x) +state.ballPos.y
            if ball_bool and state.ballPos.x < 0 and fabs(botPos_y)<1.2*HALF_FIELD_MAXY:
                #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                #print ("Bot placed at ......................................",-HALF_FIELD_MAXX*19.5/20, botPos_y)
                self.sParams.GoToPointP.x = -HALF_FIELD_MAXX
                self.sParams.GoToPointP.y =  botPos_y
                self.sParams.GoToPointP.finalVelocity = 0
                self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos) 
                sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
                #print("align with ball velocity ")
            elif ball_bool and state.ballPos.x < 0 and botPos_y>1.2*HALF_FIELD_MAXY:
                #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                #print ("Bot placed at ......................................",-HALF_FIELD_MAXX*19.5/20, botPos_y)
                self.sParams.GoToPointP.x = -HALF_FIELD_MAXX
                self.sParams.GoToPointP.y =  HALF_FIELD_MAXY
                self.sParams.GoToPointP.finalVelocity = 0
                self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos) 
                sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
                #print("align with ball velocity ")

            elif ball_bool and state.ballPos.x < 0 and botPos_y<-1.2*HALF_FIELD_MAXY:
                #print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
                #print ("Bot placed at ......................................",-HALF_FIELD_MAXX*19.5/20, botPos_y)
                self.sParams.GoToPointP.x = -HALF_FIELD_MAXX
                self.sParams.GoToPointP.y = -HALF_FIELD_MAXY
                self.sParams.GoToPointP.finalVelocity = 0
                self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos) 
                sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
                #print("align with ball velocity ")                
        if state.ballPos.x >  -HALF_FIELD_MAXX*(pi/(2*exp(1))):
            ##print(fabs(state.ballPos.y) , HALF_FIELD_MAXY*8/9)
            if fabs(state.ballPos.y) < HALF_FIELD_MAXY*8/9: 
                self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*19.5/20
                self.sParams.GoToPointP.y = -1*((OUR_GOAL_MAXY/2.5)*state.ballPos.y/HALF_FIELD_MAXY)
                self.sParams.GoToPointP.finalVelocity = 0
                self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos)

                #print("Adjusting.................",-1*(OUR_GOAL_MAXY*state.ballPos.y/GOALIE_MAXY),state.ballPos.y)
                sGoToPoint.execute(self.sParams, state, self.bot_id, pub)

        # if(sqrt(self.ballVel.absSq(self.ballVel)) < 0.02*MAX_BOT_SPEED and fabs(state.ballPos.y) < OUR_GOAL_MAXY*1.1 and fabs(state.ballPos.x) > fabs(HALF_FIELD_MAXX -2 * DBOX_WIDTH- HALF_FIELD_MAXX/7.0)):
        #    if (dist >= DRIBBLER_BALL_THRESH):
        #        self.sParams.GoToPointP.x = state.ballPos.x
        #        self.sParams.GoToPointP.y = state.ballPos.y
        #        self.sParams.GoToPointP.finalVelocity = 0
        #        self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.bot_id].y) / (state.ballPos.x - state.homePos[self.bot_id].x)))
        #        sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
               
        #    else:
        #        self.sParams.KickP.power = 7.0
        #        sKick.execute(self.sParams, state , self.bot_id,pub)
        

        elif state.ballPos.x < -HALF_FIELD_MAXX + DBOX_WIDTH and fabs(state.ballPos.y) < OUR_GOAL_MAXY and state.ballPos.x > -HALF_FIELD_MAXX:
            self.sParams.GoToPointP.x = state.ballPos.x
            self.sParams.GoToPointP.y = state.ballPos.y
            self.sParams.GoToPointP.finalVelocity = 0
            self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos)
            #print("go to ball",state.ballPos.x,state.ballPos.y)
            sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
        
        if self.ballPos.dist(self.botPos) < DRIBBLER_BALL_THRESH:
            self.sParams.KickToPointP.x = 0
            self.sParams.KickToPointP.y = 0
            self.sParams.KickToPointP.power = 7
            sKickToPoint.execute(self.sParams, state, self.bot_id, pub)

            print("_________________________KICK______________")


            threat = self.threat_with_ball(state)
            #print("threat .............",threat)
            # if (threat is not -1):
            #     threat_theta = state.awayPos[threat].theta
            #     #print("threat theta",threat_theta,"threat ",threat)
            #     self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*19/20
            #     self.sParams.GoToPointP.y = 0
            #     self.sParams.GoToPointP.finalVelocity = 0
            #     self.sParams.GoToPointP.finalslope = self.ballPos.angle(self.botPos)
            #     sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
            


            
   

        '''    
        default_x = HALF_FIELD_MAXX + SHIFT
        
        if(state.ballPos.x >  (HALF_FIELD_MAXX - SHIFT) and state.ballPos.x < 4*HALF_FIELD_MAXX/5):
            self.goalieTarget.x = int(HALF_FIELD_MAXX -SHIFT)
        else:
            self.goalieTarget.x  = int(default_x)
        
        striker = -1
        striker_dist = inf


        for oppID in xrange(5):
            if oppID == self.bot_id :
                continue
            oppPos = Vector2D(int(state.homePos[oppID].x), int(state.homePos[oppID].y))
            kick_range_test = sqrt(oppPos.absSq(oppPos-self.ballPos))

            if(kick_range_test < KICK_RANGE_THRESH and kick_range_test < striker_dist):
                    striker = oppID
                    striker_dist = kick_range_test
        if(striker is not -1):
            self.goalieTarget.y = int( ((state.ballPos.y - state.homePos[striker].y) / (state.ballPos.x - state.homePos[striker].x)) * (goalieTarget.x - state.ballPos.x) ) + state.ballPos.y
        else :
            if(state.ballVel.x == 0):
                self.goalieTarget.y = int(state.ballPos.y)
            else:
                if state.ballVel.x > 0:
                    self.goalieTarget.y = int((( state.ballVel.y / state.ballVel.x ) * ( self.goalieTarget.x \
                    - state.ballPos.x ) ) + state.ballPos.y)
                else:
                    self.goalieTarget.y = 0
        if self.goalieTarget.y < OUR_GOAL_MINY/1.2 :
            self.goalieTarget.y = int(OUR_GOAL_MINY/1.2)
        elif self.goalieTarget.y > OUR_GOAL_MAXY/1.2 :
            self.goalieTarget.y = int(OUR_GOAL_MAXY/1.2)

        self.sParams.GoToPointP.x = self.goalieTarget.x
        self.sParams.GoToPointP.y = self.goalieTarget.y
        self.sParams.GoToPointP.finalVelocity = 0
        self.sParams.GoToPointP.finalslope = atan((state.ballPos.y - state.homePos[self.bot_id].y) / (state.ballPos.x - state.homePos[self.bot_id].x))
        sGoToPoint.execute(self.sParams, state, self.bot_id, pub)
        '''

    def isComplete(self, state):
        return False

    def updateParams(self, state):
        pass
