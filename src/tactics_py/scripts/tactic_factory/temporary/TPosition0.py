
from tactic import Tactic
import time
import sys

sys.path.append('/home/catkin_ws/src/skills_py/scripts/skills')
sys.path.append('/home/catkin_ws/src/plays_py/scripts/utils/')
sys.path.insert(0, '/home/catkin_ws/src/navigation_py/scripts/navigation')

from geometry import Vector2D 
import skills_union
'''
import sGoToPoint
import sGoToBall
import sDribbleTurn
'''
import obstacle
import sDribbleTurn
import sDefendPoint
import sGoToBall
#import sTurnToPoint
import sKickToPoint

class TPosition(Tactic):

    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()

        #self.destination = Vector2D(int(self.param.PositionP.x), int(self.param.PositionP.y))
        self.threshold = 20.0

    def execute(self, state, pub):
        '''
        for sDribbleTurn--------------------------------------------------
        self.sParam.DribbleTurnP.x             = 150 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 150#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 15  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 25 #self.param.PositionP.finalSlope = 0
        '''

        #for sGoToBall----------------------------------------------------         
        self.sParam.GoToBallP.intercept      = False #true or false ask?

        
        '''
        #for sTurnToPoint------------------------------------------------
        self.sParam.TurnToPointP.x             = 200 #self.param.PositionP.x 
        self.sParam.TurnToPointP.y             = 800 #self.param.PositionP.y 
        self.sParam.TurnToPointP.max_omega     = 10  #self.param.PositionP.align = False

        

        #for sDefendPoint-------------------------------------------------
        self.sParam.DefendPointP.x             = 150 #self.param.PositionP.x 
        self.sParam.DefendPointP.y             = 150#self.param.PositionP.y 
        self.sParam.DefendPointP.finalslope    = 15  #self.param.PositionP.align = False
        self.sParam.DefendPointP.radius        = 1.2 #self.param.PositionP.finalSlope = 0

        

        #for sKickToPoint---------------------------------------------------
        self.sParam.KickToPointP.x             = 0
        self.sParam.KickToPointP.y             = 0
        self.sParam.KickToPointP.power         = 50
        '''
        '''
        self.sParam.DribbleTurnP.x             = -1200 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 2000#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 3  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 50 #self.param.PositionP.finalSlope = 0
        
        self.sParam.KickToPointP.x             = 1200
        self.sParam.KickToPointP.y             = -700
        self.sParam.KickToPointP.power         = 50
        '''
        '''
        self.sParam.DribbleTurnP.x             = 150 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 150#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 15  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 5 #self.param.PositionP.finalSlope = 0
        '''
        for i in xrange(6):
            self.bot_id = i

            sGoToBall.execute(self.sParam, state, self.bot_id, pub)

    def isComplete(self, state):
        botpos = Vector2D(int(state.homePos[bot_id].x),int(state.homePos[bot_id].y))
        ballpos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        dis_from_point = math.sqrt(math.pow(ballpos.x-botpos.x,2)+math.pow(ballpos.y-botpos.y,2))
        angle = ballpos.normalizeAngle(ballpos.angle(botpos) - (state.homePos[botID].theta))

        if (dis_from_point < 3*BOT_POINT_THRESH) and (math.fabs(angle)<0.2):
            return True
        else:
            return False

        '''
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False
        '''

    def updateParams(self, state):
        # No parameter to update here
        pass
















# from tactic import Tactic
# import time
# import sys

# sys.path.append('../../../skills_py/scripts/skills')
# sys.path.append('../../../plays_py/scripts/utils/')
# sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
# sys.path.insert(0, '../../../navigation_py/scripts/navigation')

# from geometry import * 
# import skills_union
# from config import *
# import obstacle
# import sGoToPoint
# import sStop
# from tactic import Tactic
# from geometry import Vector2D
# import skills_union
# import sKick
# import sKickToPoint
# import sGoToBall
# from math import *
# from numpy import inf
# from config import *
# MAX_DRIBBLE_RANGE = 3
# KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
# THRES = 0.8
# THETA_THRESH = 0.005
# GOALIE_MAXX = 2450
# GOALIE_MAXY =650

# SHIFT = (HALF_FIELD_MAXX/7.0)

# class TPosition(Tactic):
#     def __init__(self,botID,state,params=None):
#         self.botID = botID
#         self.ballAim = Vector2D()
#         self.goalieTarget = Vector2D(0,0)
#         self.ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
#         self.botPos = Vector2D(int(state.homePos[botID].x), int(state.homePos[botID].y))
#         self.ballVel = Vector2D(int(state.ballVel.x) , int(state.ballVel.y))
#         self.sParams = skills_union.SParam()
        
#         #placing the goalie


#     def ball_velocity_direction(self,state):
#         ballVel = Vector2D(int(state.ballVel.x), int(state.ballVel.y))
#         ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

#         Upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MAXY / 2 + BOT_BALL_THRESH))
#         Lower_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MINY / 2 - BOT_BALL_THRESH))

#         alpha_1 = atan((Upper_limit.y - ballPos.y)/ (ballPos.x - Upper_limit.x)) 
#         alpha_2 = atan((Lower_limit.y - ballPos.y)/ (ballPos.x - Lower_limit.x))

#         if ballVel.x < 0:
#             ball_vel_angle = atan(state.ballVel.y*-1 / state.ballVel.x)
#         else:
#             return False,inf
#         #deciding the direction of the ball's velocity
#         if ballVel.y > 0:
#             if(ball_vel_angle > alpha_2 and ball_vel_angle < alpha_1):
#                 return True,ball_vel_angle
#             else :
#                 return False,inf
#         else:
#             if(abs(ball_vel_angle) > abs(alpha_1) and abs(ball_vel_angle) < abs(alpha_2)):
#                 return True,ball_vel_angle
#             else:
#                 return False,inf

#     def threat_with_ball(self,state):
#         threat = -1
#         away_in_our_side_with_ball = []
#         ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
#         for away_botID in xrange(len(state.awayPos)):
#             if state.awayPos[away_botID].x < 0 and ballPos.dist(state.awayPos[away_botID]) <= BOT_RADIUS*1.5 :
#                 away_in_our_side_with_ball.append(away_botID)
#         if len(away_in_our_side_with_ball) > 0:
#             return away_in_our_side_with_ball[0]
#         else :
#             return -1

#     def determine_line(Y,X,m,x):
#         y = m*(x-X) +Y




#     def execute(self, state , pub):
#         for i in xrange(6):
#             self.botID=i
#             dist = self.ballPos.dist(self.botPos)
#             '''     
#             if(sqrt(self.ballVel.absSq()) < 0.02*MAX_BOT_SPEED and  fabs(state.ballPos.y) < OUR_GOAL_MAXY and state.ballPos.x > (HALF_FIELD_MAXX- (2 * DBOX_WIDTH) (HALF_FIELD_MAXX/5.0)):
#             '''
            
#             #sprint("printing ................",bot_x,bot_y,HALF_FIELD_MAXX*-3.5/5)
#             #print(len(state.homeVel),"fddddddddddddddddddddddddd")
            
#             if not (fabs(state.homePos[self.botID].y) < OUR_GOAL_MAXY*1.1 and state.homePos[self.botID].x <= HALF_FIELD_MAXX*-3.5/5) :

#                 #self.sParams.GoToPointP.x = state.homePos[self.botID].x
#                 self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*19.5/20
#                 self.sParams.GoToPointP.y = 0
#                 #if state.homePos[self.botID].y < 0:
#                 #    self.sParams.GoToPointP.y = OUR_GOAL_MINY
#                 #else:
#                 #    self.sParams.GoToPointP.y = OUR_GOAL_MAXY
#                 self.sParams.GoToPointP.finalVelocity = 0
#                 self.sParams.GoToPointP.finalslope    = 0
#                 print("beyond y and x.................................",self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
#                 sGoToPoint.execute(self.sParams, state, self.botID, pub)
#                 print("............................y and x...")
           

#             #re-placing the bot 

#             elif not (state.homePos[self.botID].x < HALF_FIELD_MAXX*-3.5/5) :

#                 self.sParams.GoToPointP.x = -HALF_FIELD_MAXX
#                 self.sParams.GoToPointP.y = 0
                
#                 self.sParams.GoToPointP.finalVelocity = 0
#                 self.sParams.GoToPointP.finalslope = 0
#                 print("beyond x...................................",self.sParams.GoToPointP.x,self.sParams.GoToPointP.y)
#                 sGoToPoint.execute(self.sParams, state, self.botID, pub)
#                 print (".......................x..")

#             #elif fabs(state.homePos[self.botID].x+GOALIE_MAXX) < 150 and fabs(state.homePos[self.botID].y) < 80 and  (state.homeVel[self.botID].x >= 0 or state.homeVel[self.botID].y >= 0) :
#                 #print "shubham stop"+"**"*50
#                 #  sStop.execute(self.sParams, state, self.botID, pub)
                
            
#             #print(state.ballPos.x)

#             ball_bool, angle = self.ball_velocity_direction(state)
#             if ball_bool and state.ballPos.x < 0:
#                 botPos_y = tan(angle) * (-HALF_FIELD_MAXX - state.ballPos.x) +state.ballPos.y
#                 print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
#                 print ("BOt placed at ......................................",-HALF_FIELD_MAXX*19.5/20, botPos_y)
#                 self.sParams.GoToPointP.x = -HALF_FIELD_MAXX
#                 self.sParams.GoToPointP.y =  botPos_y
#                 self.sParams.GoToPointP.finalVelocity = 0
#                 self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))) 
#                 sGoToPoint.execute(self.sParams, state, self.botID, pub)
#                 print("align with ball velocity ")

#             if state.ballPos.x >  0:
#                 #print(fabs(state.ballPos.y) , HALF_FIELD_MAXY*8/9)
#                 if fabs(state.ballPos.y) < HALF_FIELD_MAXY*8/9: 
#                     self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*19.5/20
#                     self.sParams.GoToPointP.y = ((OUR_GOAL_MAXY/2.5)*state.ballPos.y/HALF_FIELD_MAXY)*-1
#                     self.sParams.GoToPointP.finalVelocity = 0
#                     self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))) 

#                     print("Adjusting.................",-1*(OUR_GOAL_MAXY*state.ballPos.y/GOALIE_MAXY),state.ballPos.y)
#                     sGoToPoint.execute(self.sParams, state, self.botID, pub)

            
#             elif state.ballPos.x < 0:
#                 #print(int(state.ballPos.x))
                
#                 if self.ballPos.dist(state.homePos[self.botID]) < DRIBBLER_BALL_THRESH :
#                     #self.sParams.KickToPointP.x = 0
#                     #self.sParams.KickToPointP.y = 0
#                     #self.sParams.KickToPointP.power     = 7
#                     self.sParams.KickP.power = 7
#                     #sKickToPoint.execute(self.sParams, state, self.botID, pub)
                    
#                     print("kick")
#                     sKick.execute(self.sParams, state , self.botID,pub)

#                 elif state.ballPos.x < -HALF_FIELD_MAXX*3.5/5  and fabs(state.ballPos.y) < fabs(OUR_GOAL_MAXY*1.1) :
#                     self.sParams.GoToPointP.x = state.ballPos.x
#                     self.sParams.GoToPointP.y = state.ballPos.y
#                     self.sParams.GoToPointP.finalVelocity = 0
#                     self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))) 
#                     print("go to ball",state.ballPos.x,state.ballPos.y)
#                     sGoToPoint.execute(self.sParams, state, self.botID, pub)
#                     print("at ball")


#                 threat = self.threat_with_ball(state)
#                 #print("threat .............",threat)
#                 if (threat is not -1):
#                     threat_theta = state.awayPos[threat].theta
#                     print("threat theta",threat_theta,"threat ",threat)
#                     self.sParams.GoToPointP.x = -HALF_FIELD_MAXX*19.5/20
#                     self.sParams.GoToPointP.y = 0
#                     self.sParams.GoToPointP.finalVelocity = 0
#                     self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x)))
#                     sGoToPoint.execute(self.sParams, state, self.botID, pub)
                
            
            

            
            

        
                
                    
        
            


#             '''
#                     self.sParams.KickToPointP.x = 0
#                     self.sParams.KickToPointP.y = 0
#                     self.sParams.KickToPointP.power     = 7
#                     print("kick 00")
#                     sKickToPoint.execute(self.sParams, state, self.botID, pub)
            
            
#             else:
#                 self.sParams.GoToPointP.x = -GOALIE_MAXX
#                 self.sParams.GoToPointP.y =  0
#                 self.sParams.GoToPointP.finalVelocity = 0
#                 sGoToPoint.execute(self.sParams , state , self.botID , pub)
            

            
#             '''

                
#             '''    
#                 if(sqrt(self.ballVel.absSq(self.ballVel)) < 0.02*MAX_BOT_SPEED and fabs(state.ballPos.y) < OUR_GOAL_MAXY*1.1 and fabs(state.ballPos.x) > fabs(HALF_FIELD_MAXX -2 * DBOX_WIDTH- HALF_FIELD_MAXX/7.0)):
#                    if (dist >= DRIBBLER_BALL_THRESH):
#                        self.sParams.GoToPointP.x = state.ballPos.x
#                        self.sParams.GoToPointP.y = state.ballPos.y
#                        self.sParams.GoToPointP.finalVelocity = 0
#                        self.sParams.GoToPointP.finalslope = self.ballPos.normalizeAngle(atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x)))
#                        sGoToPoint.execute(self.sParams, state, self.botID, pub)
                       
#                    else:
#                        self.sParams.KickP.power = 7.0
#                        sKick.execute(self.sParams, state , self.botID,pub)
#             '''   

#             '''    
#             default_x = HALF_FIELD_MAXX + SHIFT
            
#             if(state.ballPos.x >  (HALF_FIELD_MAXX - SHIFT) and state.ballPos.x < 4*HALF_FIELD_MAXX/5):
#                 self.goalieTarget.x = int(HALF_FIELD_MAXX -SHIFT)
#             else:
#                 self.goalieTarget.x  = int(default_x)
            
#             striker = -1
#             striker_dist = inf


#             for oppID in xrange(5):
#                 if oppID == self.botID :
#                     continue
#                 oppPos = Vector2D(int(state.homePos[oppID].x), int(state.homePos[oppID].y))
#                 kick_range_test = sqrt(oppPos.absSq(oppPos-self.ballPos))

#                 if(kick_range_test < KICK_RANGE_THRESH and kick_range_test < striker_dist):
#                         striker = oppID
#                         striker_dist = kick_range_test
#             if(striker is not -1):
#                 self.goalieTarget.y = int( ((state.ballPos.y - state.homePos[striker].y) / (state.ballPos.x - state.homePos[striker].x)) * (goalieTarget.x - state.ballPos.x) ) + state.ballPos.y
#             else :
#                 if(state.ballVel.x == 0):
#                     self.goalieTarget.y = int(state.ballPos.y)
#                 else:
#                     if state.ballVel.x > 0:
#                         self.goalieTarget.y = int((( state.ballVel.y / state.ballVel.x ) * ( self.goalieTarget.x \
#                         - state.ballPos.x ) ) + state.ballPos.y)
#                     else:
#                         self.goalieTarget.y = 0
#             if self.goalieTarget.y < OUR_GOAL_MINY/1.2 :
#                 self.goalieTarget.y = int(OUR_GOAL_MINY/1.2)
#             elif self.goalieTarget.y > OUR_GOAL_MAXY/1.2 :
#                 self.goalieTarget.y = int(OUR_GOAL_MAXY/1.2)

#             self.sParams.GoToPointP.x = self.goalieTarget.x
#             self.sParams.GoToPointP.y = self.goalieTarget.y
#             self.sParams.GoToPointP.finalVelocity = 0
#             self.sParams.GoToPointP.finalslope = atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))
#             sGoToPoint.execute(self.sParams, state, self.botID, pub)
#             '''

#     def isComplete(self, state):
#         return False

#     def updateParams(self, state):
#         pass
