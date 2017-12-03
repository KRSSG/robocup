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
import sGoToPoint
import numpy as np
from numpy import array,inf
from isPossible import isPossible

KICK_RANGE_THRESH = MAX_DRIBBLE_R   #ASK
THRES  = 0.8
THETA_THRESH = 0.005
TURNING_THRESH = 10
# k=k2=50000 no threshold
THRESH = pi**2 /180
BOT_OPP_DIST_THRESH =  500

ROWS                                    =  33
COLS                                    =  22
OPTIMAL_DISTANCE                        =  3.0*HALF_FIELD_MAXX/5
PASSING_PARAMETER                       =  5
BISECTOR_CONST                          =  70
OUR_BOT_APPROACHABILITY_CONST           =  35 
OPTIMAL_DISTANCE_CONST                  =  10
AWAY_BOT_APP_GWT                        =  30
BOT_ORIENT_CONST                        =  0.2
KICK_DISTANCE_CONST                     =  90
PASS_PROB_THRESH                        =  2.0
k=k2=20*pi/180

class TAttackerX(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TAttackerX, self).__init__( bot_id, state, param)
        self.bot_id = bot_id
        self.sParams = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MINY)

        self.GOAL_UPPER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MAXY*3)
        self.GOAL_LOWER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY*3)


    def bot_orient_wt(self,state, botID):
        orient_angles= []
        # deviation=[]
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        no_use = Vector2D()
        for our_bot in xrange(len(state.homePos)):
            if our_bot is botID:
                continue
            #print(our_bot)
            angle_needed = no_use.normalizeAngle(fabs(state.homePos[our_bot].theta-state.homePos[botID].theta)-pi )
            # deviation.append([our_bot, normalizeDistance])
            
            normalizedAngle  = BOT_ORIENT_CONST * exp(-1*fabs(angle_needed)/ pi)
            orient_angles.append([our_bot, normalizedAngle])

        return orient_angles
    def velocity_dir_wt(self,state,botID):
        #not completed
        wt = [[i,0] for i in xrange(5)]
        return wt
    def optimal_dist_wt(self,state,botID):
        
        deviation = []
        AttackPos = Vector2D(int(state.homePos[botID].x), int(state.homePos[botID].y))
        for our_bot in xrange(len(state.homePos)):
            if our_bot is botID:
                continue
            distance = fabs(Vector2D(int(state.homePos[our_bot].x),int(state.homePos[our_bot].y)).dist(Vector2D(int(state.homePos[botID].x),int(state.homePos[botID].y))))
            normalizeDistance = OPTIMAL_DISTANCE_CONST * exp(-1*fabs((OPTIMAL_DISTANCE - distance))/HALF_FIELD_MAXY)
            deviation.append([our_bot, normalizeDistance])
        return deviation

    def ourBot_approachability_wt(self, state, botID):
        ourBot_app_wt = []

        for our_bot in xrange(len(state.homePos)):
            if our_bot is botID:
                continue

            distance = fabs(Vector2D(int(state.homePos[our_bot].x),int(state.homePos[our_bot].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            normalizeDistance = OUR_BOT_APPROACHABILITY_CONST * exp(-1*distance/HALF_FIELD_MAXY)
            ourBot_app_wt.append([our_bot, normalizeDistance])
        ##print(ourBot_app_wt)
        ##print("bod_id  ",self.bot_id)
        return ourBot_app_wt

    
                

    def calculate_score(self, state,botID):

        weights = [0.7, 0.8, 0.16, 0.4]


        score_orient   = self.bot_orient_wt(state, botID)
        score_opt_dist = self.optimal_dist_wt(state, botID)
        score_bot_app  = self.ourBot_approachability_wt(state, botID)
        #score_velo_dir = self.velocity_dir_wt(state)



        # #print "score opt distance: "+str(score_opt_dist[1])

        score = []
        #print(score_orient)
        max_goal_score = 0
        possibility_recv = isPossible(botID, state)
        for i in xrange(len(state.homePos)-1):
            # if i is botID:
            #     continue
            botId = score_orient[i][0]
            possibility             = isPossible(botId, state)
            bool_goal,pt,score_goal = possibility.isGoalPossible(state)

            if score_goal > max_goal_score:
                max_goal_score = score_goal

            if bool_goal:
                goal_score = exp(score_goal-0.45)
            else:
                goal_score = 0

            print(botId,goal_score,score_goal)
            bot_score =  score_orient[i][1]*weights[0] + score_opt_dist[i][1]*weights[1] + score_bot_app[i][1]*weights[2] + goal_score*weights[3]
            if state.homePos[score_orient[i][0]].x < -HALF_FIELD_MAXX*0.5:
                bot_score = 0

            bool_receive = possibility_recv.isReceivePossible(state, botId)
            bool_pass    = possibility_recv.isPassPossible(state, botId) 

            
            if state.homePos[botId].x < -HALF_FIELD_MAXX*0.5:
                bot_score = 0
            # print(botId, bot_score, state.homePos[botId].x , -HALF_FIELD_MAXX*0.5)
            #print(botId, bool_pass,bool_receive,bool_goal)
            score.append([botId, bot_score])

        score.sort(key = lambda x:x[1], reverse = True)
        return score, max_goal_score
    

    def execute(self, state, pub, receiver_bot_id = [-1], goal = [False]):
        #score=np.zeros((ROWS,COLS),dtype=np.float)+-
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        DistancesFromBot = inf
        # count=0
        # for i in state.homePos:
        #     botIDs=Vector2D(int(i.x),int(i.y))
        #     dist=botIDs.dist(ballPos)
        #     if dist<DistancesFromBot:
        #         DistancesFromBot=dist
        #         self.bot_id=count
        #     count+=1

        
        import sKickToPoint
        import sGoToBall
        #print("yo i m hgksdgsdjkfgdig attacker ",self.bot_id)

        if not (state.ballPos.x < -HALF_FIELD_MAXX + DBOX_WIDTH  and  fabs(state.ballPos.y) < OUR_GOAL_MAXY*0.8) :
            if fabs(ballPos.dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y)))>BOT_BALL_THRESH):
                sGoToBall.execute(self.sParams, state, self.bot_id, pub)
        else:
            return

        possibility = isPossible(self.bot_id,state)
        bool_goal,point,score_goal = possibility.isGoalPossible(state)
        

        scores_of_bot, mid_fielder_score = self.calculate_score(state,self.bot_id)
        if bool_goal :
            if score_goal > mid_fielder_score or 1:
                print("\n\n_______FUCK__GOALIE___________\n\n")
                self.sParams.KickToPointP.x              = point.x
                self.sParams.KickToPointP.y              = point.y
                self.sParams.KickToPointP.power          = 7.0
                sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
                goal[0] = True
                return
            else:
                goal[0] = False



        choosen_bot = scores_of_bot[0][0]
        if scores_of_bot[0][1] < 2 :
            print(scores_of_bot[0][1])
            return

        
        print(" ")
        print("_______         _________",choosen_bot,"______         _______")
        print(" ")

        if possibility.isPassPossible(state, choosen_bot):
            #print("Passing.........................................................",self.bot_id,choosen_bot)
            if possibility.isReceivePossible(state, choosen_bot):
                #print("Receiving..............................................................",self.bot_id,choosen_bot)
                receiver_bot_id[0] = choosen_bot
                self.sParams.KickToPointP.x = state.homePos[choosen_bot].x       
                self.sParams.KickToPointP.y = state.homePos[choosen_bot].y
                self.sParams.KickToPointP.power         = 7
                sKickToPoint.execute(self.sParams, state, self.bot_id, pub)




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

