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

class TAttack(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TAttack, self).__init__( bot_id, state, param)
        self.bot_id = bot_id
        self.sParams = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MINY)

        self.GOAL_UPPER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MAXY*3)
        self.GOAL_LOWER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY*3)


    def Dist(self,a,state):
        a=a[0]
        return sqrt(pow((a.x-state.homePos[self.bot_id].x),2)+ pow((a.y-(state.homePos[self.bot_id].y)),2))
    def Dist2(self,a,state):
        return a[0]
    def Dist3(self,a,state):
        return fabs(a[0].x-state.homePos[self.bot_id].x)+ fabs(a[0].y-int(state.homePos[self.bot_id].y))
    
    def DistTheta(self,thetaLine, thetaBot, BotPos, AttackPos):
        thetaleft=thetaLine-thetaBot
        distance=AttackPos.dist(BotPos)
        return sin(thetaleft)*distance

    def bisector_bot_wt(self,state):
        
        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        DistancesFromBot=999999999999
        count=0
        closest_opp=0
        for i in state.awayPos:
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(ballPos)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                closest_opp=count
            count+=1

     
        opponents=[]
        j=0
        for i in state.awayPos:
            opponents.append([Vector2D(int(i.x),int(i.y)),j])
            j+=1
        opponents.sort(key=lambda opp: self.Dist(opp,state))
        #comment out opponents=opponents[0:4]
        angles=[]
        for opponent in opponents:
            #print "x: "+str(opponent[0].x)+" y: "+str(opponent[0].y)
            #print "my theta of :"+str(self.bot_id)+" is "+str((state.homePos[self.bot_id].theta)/pi*180)
            angle=opponent[0].normalizeAngle(opponent[0].angle(botPos) - (state.homePos[self.bot_id].theta))
            #print "opp theta of :"+str(opponent[1])+" is "+str(angle*180/pi)
            angles.append([angle,opponent[1]])
        angles.sort(key=lambda x: x[0])
        #print "angles are: "+str(angles)
        self.Opp_sorted_Angles=[]
        for angle in angles:
            self.Opp_sorted_Angles.append([angle[0]-THRESH,angle[0]+THRESH,angle[1]])
        lineangles=[]
        angle_gap=18.314*pi/180
        angle_to_shoot=[]
        i=0; j=0
        weights=[]
        #print "Opp angles: "+str(self.Opp_sorted_Angles)


        # for i in xrange(len(self.Opp_sorted_Angles)):
        #     origin_angle=self.Opp_sorted_Angles[i][1]
        #     j=i+1
        #     if j>=len(self.Opp_sorted_Angles): j=0
        #     if self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]>angle_gap:
        #         angle_here=self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]
        #         if angle_here>pi:
        #             angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
        #         else:
        #             angle_to_shoot.append(origin_angle+angle_here/2)


        for i in xrange(len(self.Opp_sorted_Angles)):
            origin_angle=self.Opp_sorted_Angles[i][1]
            j=i+1
            flag=0
            if j>=len(self.Opp_sorted_Angles): j=0; flag=1
            diff=self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]
            
            if flag:
                diff*=-1
                diff = 2*pi -diff


            if (diff>angle_gap) :
                angle_here=diff
                #print "angle here inside: "+str(array(angle_here)*(180/pi))
                if angle_here>pi:
                    angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
                else:
                    angle_to_shoot.append(origin_angle+angle_here/2) 

        #print("Angle to shoot "+ str(array(angle_to_shoot)*(180/pi)))

        for i in xrange(len(state.homePos)):
            if i==self.bot_id: continue
            dist=fabs(Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            angle_here=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)).angle(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))) - state.homePos[self.bot_id].theta
            min_dist= np.inf
            # print "angles to shoot "+str(angle_to_shoot)
            for angles in angle_to_shoot:
                angleneeded=fabs(angle_here-angles)
                min_dist_here=dist*sin(angleneeded)
                if min_dist_here<min_dist:
                    min_dist=min_dist_here
            normalizeDistance_min = BISECTOR_CONST * exp(-1*min_dist/HALF_FIELD_MAXY)
            normalizeDistance = BISECTOR_CONST * exp(-1*dist/HALF_FIELD_MAXY)
            weights.append([i,normalizeDistance_min, normalizeDistance])


        self.bots_to_remove=[]
        # goal_angle_upper = botPos.normalizeAngle(Vector2D(int(self.UPPER_HALF.x), int(self.UPPER_HALF.y)).angle(botPos) - state.homePos[self.bot_id].theta)#an2(UPPER_HALF.y-state.homePos[self.bot_id].y, UPPER_HALF.x-state.homePos[self.bot_id].x)
        # goal_angle_lower = botPos.normalizeAngle(Vector2D(int(self.LOWER_HALF.x), int(self.LOWER_HALF.y)).angle(botPos) - state.homePos[self.bot_id].theta)
        # if (fabs(goal_angle_upper-goal_angle_lower))>=pi:
        #     goal_angle_upper,goal_angle_lower=goal_angle_lower,goal_angle_upper
        # print "goal angle lower : "+str(goal_angle_lower*180/pi)
        # print "goal angle upper: "+str(goal_angle_upper*180/pi)     
        # #goal_angle_lower = atan2(LOWER_HALF.y-state.homePos[self.bot_id].y, LOWER_HALF.x-state.homePos[self.bot_id].x)
        for i in xrange(len(state.homePos)):
            if i==self.bot_id: continue
            myBotPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            angle=myBotPos.angle(botPos)-state.homePos[self.bot_id].theta
            #print "angle for bot "+str(i)+" is "+str(angle*180/pi)
            for angles in self.Opp_sorted_Angles:
    
                if (angle>=angles[0] and angle<=angles[1])  :

                    dist_of_homeBot=myBotPos.dist(botPos)
                    dist_of_awayBot=Vector2D(int(state.awayPos[angles[2]].x),int(state.awayPos[angles[2]].y)).dist(botPos)
                   
                    if fabs(dist_of_awayBot)<fabs(dist_of_homeBot)+BOT_OPP_DIST_THRESH:
                        self.bots_to_remove.append(i)
                

        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        goal_angle_upper = atan((self.UPPER_HALF.y-botPos.y)*1.0/ (self.UPPER_HALF.x-botPos.x))

        goal_angle_lower = atan((self.LOWER_HALF.y-botPos.y)*1.0/ (self.LOWER_HALF.x-botPos.x))

        #print("signs ",(self.UPPER_HALF.y-botPos.y)* (self.UPPER_HALF.x-botPos.x), (self.LOWER_HALF.y-botPos.y)* (self.LOWER_HALF.x-botPos.x))
        #goal_angle_upper,goal_angle_lower = map(botPos.normalizeAngle, (goal_angle_upper,goal_angle_lower))
        #print(goal_angle_lower,goal_angle_upper,"fhasdkfasdk")
        
        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            bot_angle = atan((state.homePos[our_bot].y - botPos.y)/ (state.homePos[our_bot].x - botPos.x))
            #print("bot angle", bot_angle ,our_bot)
            if bot_angle > goal_angle_upper and bot_angle < goal_angle_lower :
                self.bots_to_remove.append(our_bot)

        #print("bots to remove            ", self.bots_to_remove)

        return weights


    def bot_orient_wt(self,state):
        orient_angles= []
        # deviation=[]
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        no_use = Vector2D()
        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            angle_needed = no_use.normalizeAngle(state.homePos[our_bot].theta-state.homePos[self.bot_id].theta)
            # deviation.append([our_bot, normalizeDistance])
            if fabs(angle_needed) < 90 :
                parameter = fabs(angle_needed) * BOT_ORIENT_CONST
            else:
                parameter = -1*fabs(angle_needed) * BOT_ORIENT_CONST
            orient_angles.append([our_bot, parameter])

        return orient_angles
    def velocity_dir_wt(self,state):
        #not completed
        wt = [[i,1] for i in xrange(5)]
        return wt
    def optimal_dist_wt(self,state):
        
        deviation = []
        AttackPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            distance = fabs(Vector2D(int(state.homePos[our_bot].x),int(state.homePos[our_bot].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            normalizeDistance = OPTIMAL_DISTANCE_CONST * exp(-1*(OPTIMAL_DISTANCE - distance)/HALF_FIELD_MAXY)
            deviation.append([our_bot, normalizeDistance])
        return deviation

    def ourBot_approachability_wt(self, state):
        ourBot_app_wt = []

        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            distance = fabs(Vector2D(int(state.homePos[our_bot].x),int(state.homePos[our_bot].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            normalizeDistance = OUR_BOT_APPROACHABILITY_CONST * exp(-1*distance/HALF_FIELD_MAXY)
            ourBot_app_wt.append([our_bot, normalizeDistance])
        #print(ourBot_app_wt)
        #print("bod_id  ",self.bot_id)
        return ourBot_app_wt

    
                

    def calculate_score(self, state,BOTiD):

        weights = [0.35, 0.25, 0.15, 0.05, 0.15]
        dist_weight_partition = [0.25, 0.75]

        score_bisect   = self.bisector_bot_wt(state)
        score_orient   = self.bot_orient_wt(state)
        #score_velo_dir = self.velocity_dir_wt(state)
        score_opt_dist = self.optimal_dist_wt(state)
        score_bot_app  = self.ourBot_approachability_wt(state)



        # print "score opt distance: "+str(score_opt_dist[1])

        score = []
        # print("bisect "+str(score_bisect))
        # print("orient "+str(score_orient))
        # print("velo dir")
        our_bot=0
        for i in xrange(len(state.homePos)):
            if i is BOTiD:
                continue
            #print "score here: "+str(score_bot_app[our_bot])
            botid = score_bisect[our_bot][0]
            #print(botid)
            bot_score = (score_bisect[our_bot][1]*weights[0]*dist_weight_partition[0] + score_bisect[our_bot][2]*weights[0]*dist_weight_partition[1]) +\
            (score_orient[our_bot][1]*weights[1])+\
            (score_opt_dist[our_bot][1]*weights[3])+\
            (score_bot_app[our_bot][1]*weights[4])

            

            #####
            #(score_velo_dir[our_bot][1]*weights[2])+\
            ####
            #print((score_bot_kick[our_bot][1]*weights[5]))
            if botid in self.bots_to_remove:
                score.append([botid,0])
            else:
                score.append([botid, bot_score])
            our_bot+=1
        score.sort(reverse= True ,key = lambda x:x[1] )
        return score
    

    def opponent_approachability_gwt(self, state, grid):

        opp_app_wt = []
        opp_dist = []
        normalizeDistance = 0.0
        for opp_id in xrange(len(state.awayPos)):
            distance = fabs(Vector2D(int(state.awayPos[opp_id].x), int(state.awayPos[opp_id].y)).dist(grid.x, grid.y))
            normalizeDistance += AWAY_BOT_APP_GWT * exp(1*distance/HALF_FIELD_MAXY)
        return -1*normalizeDistance

    def gauss_dependencies(self, start, end):
        avg = 0.0

        for x in xrange(start, end):
            avg += x
        avg /= fabs(end-start+1)

        for x in xrange(start, end):
            std_dev += (x-avg)**2

        std_dev = sqrt(std_dev/ fabs(end-start+1))
        return avg, std_dev

    def gauss(self,x, mean, std_dev):   
        return (1/(sqrt(2*pi)*sqrt(std_dev))*exp(-0.5*((x-mean)/std_dev)**2))

    def scaled_lap_of_gauss(self,x, mean, std_dev):
        return (x**2-2(std_dev**2))/(std_dev**2)*exp(-0.5*(x/std_dev)**2)

    #def paasing_prob(self, state, botID):

    def goal_possible(self,state):

        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))

        #print ("chutioya saala ",botPos.x )
        goal_angle_upper = atan((self.GOAL_UPPER.y-botPos.y)*1.0/ (self.GOAL_UPPER.x-botPos.x))
        goal_angle_lower = atan((self.GOAL_LOWER.y-botPos.y)*1.0/ (self.GOAL_LOWER.x-botPos.x))
        goal_angle_mid   = 0.5*(goal_angle_lower+goal_angle_upper)

        
        k1 = False
        k2 = False
        for away_bot in xrange(len(state.awayPos)):
            
            if state.awayPos[away_bot].x > state.homePos[self.bot_id].x:
                #print (state.awayPos[away_bot].x, state.homePos[self.bot_id].x, "chutiya.....")
                away_BOT = Vector2D (int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y))
                bot_angle = atan((away_BOT.y - botPos.y)*1.0/ (away_BOT.x - botPos.x))

                #print("bhosdiwala angles ",bot_angle, goal_angle_lower, goal_angle_mid, goal_angle_upper, away_bot)
                if bot_angle > goal_angle_lower and bot_angle < goal_angle_mid:
                    k1=True
                     
                if bot_angle > goal_angle_mid and bot_angle < goal_angle_upper:
                    k2=True
                print(k1,k2)
                    
        if k1 and k2:
            return [False, -1]
        if k1 :
            return [True, 2]
        if k2 :
            return [True, 1]
        if not (k1 or k2):
            return [True, 0]


    def execute(self, state, pub):
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

        goal_chck = self.goal_possible(state)
        import sKickToPoint
        import sGoToBall
        print("yo i m hgksdgsdjkfgdig attacker ",self.bot_id)

        if fabs(ballPos.dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y)))>BOT_BALL_THRESH):
            sGoToBall.execute(self.sParams, state, self.bot_id, pub)
        flaggu=0
        possibility = isPossible(self.bot_id,state)
        bool_goal = possibility.isGoalPossible(state)

        if bool_goal[0]:
            print("_______FUCK______")
            self.sParams.KickToPointP.x = bool_goal[1].x
            self.sParams.KickToPointP.y = bool_goal[1].y
            self.sParams.KickToPointP.power          = 7.0
            sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
            flaggu=1

        if goal_chck[0]:
            print(".....FUCK GOALIE....")
            print(goal_chck)
            if goal_chck[1] == 0 :
                y_COR = 0
            if goal_chck[1] == 1 :
                y_COR = -300
            if goal_chck[1] == 2 :
                y_COR = 300
            self.sParams.KickToPointP.x = HALF_FIELD_MAXX
            self.sParams.KickToPointP.y = y_COR
            self.sParams.KickToPointP.power          = 7.0
            sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
            flaggu=1


        scores_of_bot = self.calculate_score(state,self.bot_id)
        AttackBot_ID = self.bot_id

        if scores_of_bot[0][1] - scores_of_bot[1][1] < PASSING_PARAMETER and (scores_of_bot[0][1] and scores_of_bot[1][1]):
            #self.bot_id = 
            scores_of_bot_primary = self.calculate_score(state,scores_of_bot[0][0])
            # max_primary = scores_of_bot_primary[0][1]
            score_primary=0
            for each in scores_of_bot_primary:
                if each[0]!=AttackBot_ID:
                    score_primary+=each[1]

            #self.bot_id =
            scores_of_bot_secondary = self.calculate_score(state,scores_of_bot[1][0])

            score_secondary=0
            for each in scores_of_bot_secondary:
                if each[0]!=AttackBot_ID:
                    score_secondary+=each[1]


            score_primary+=PASS_PROB_THRESH * scores_of_bot[0][1]
            score_secondary+=PASS_PROB_THRESH * scores_of_bot[1][1]

            if score_primary > score_secondary:
                print str(scores_of_bot[0][0])+" is the best bot with a score of "+str(score_primary)
            else:
                print str(scores_of_bot[1][0])+" is the best bot with a score of "+str(score_secondary)
        else:
             print str(scores_of_bot[0][0])+" is the best bot with a score of "+str(scores_of_bot[0][1])
        oh_my_bot=scores_of_bot[0][0]

        print("\n\n\n\n\n\n\n\n\n\nScore: "+str(scores_of_bot))

        if not flaggu:    
            if possibility.isPassPossible(state, oh_my_bot):
                print("Passing.........................................................",self.bot_id,oh_my_bot)
                if possibility.isReceivePossible(state, oh_my_bot):
                    print("Receiving..............................................................",self.bot_id,oh_my_bot)
                    self.sParams.KickToPointP.x = state.homePos[oh_my_bot].x       
                    self.sParams.KickToPointP.y = state.homePos[oh_my_bot].y
                    self.sParams.KickToPointP.power         = 7
                    sKickToPoint.execute(self.sParams, state, self.bot_id, pub)

        import sGoToPointOpp
        self.sParams.GoToPointP.x=(self.sParams.KickToPointP.x + state.homePos[self.bot_id].x)/2
        self.sParams.GoToPointP.y=(self.sParams.KickToPointP.y + state.homePos[self.bot_id].y)/2
        sGoToPointOpp.execute(self.sParams,state,0,pub)


        







        



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





















# from tactic import Tactic
# import time
# import sys
# from math import *

# sys.path.append('../../../skills_py/scripts/skills')
# sys.path.append('../../../plays_py/scripts/utils/')
# sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
# sys.path.insert(0, '../../../navigation_py/scripts/navigation')

# from geometry import * 
# import skills_union
# from config import *
# import obstacle
# import sGoToPoint

# KICK_RANGE_THRESH = MAX_DRIBBLE_R   #ASK
# THRES  = 0.8
# THETA_THRESH = 0.005
# TURNING_THRESH = 10
# # k=k2=50000 no threshold
# k=k2=20*pi/180


# class TPosition(Tactic):
#     def __init__(self, bot_id, state, param=None):
#         super(TPosition, self).__init__( bot_id, state, param)
#         self.sParam = skills_union.SParam()

#     def Dist(self,a,state):
#         a=a[0]
#         return sqrt(pow((a.x-state.homePos[self.bot_id].x),2)+ pow((a.y-(state.homePos[self.bot_id].y)),2))
#     def Dist2(self,a,state):
#         return a[0]
#     def Dist3(self,a,state):
#         return fabs(a[0].x-state.homePos[self.bot_id].x)+ fabs(a[0].y-int(state.homePos[self.bot_id].y))
    
#     def DistTheta(self,thetaLine, thetaBot, BotPos, AttackPos):
#         thetaleft=thetaLine-thetaBot
#         distance=AttackPos.dist(BotPos)
#         return sin(thetaleft)*distance


#     def execute(self, state, pub):
#         ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
#         DistancesFromBot=999999999999
#         count=0
#         for i in state.homePos:
#             botIDs=Vector2D(int(i.x),int(i.y))
#             dist=botIDs.dist(ballPos)
#             if dist<DistancesFromBot:
#                 DistancesFromBot=dist
#                 self.bot_id=count
#             count+=1
#         botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
#         DistancesFromBot=999999999999
#         count=0
#         closest_opp=0
#         for i in state.awayPos:
#             botIDs=Vector2D(int(i.x),int(i.y))
#             dist=botIDs.dist(ballPos)
#             if dist<DistancesFromBot:
#                 DistancesFromBot=dist
#                 closest_opp=count
#             count+=1

#         opponents=[]
#         j=0
#         for i in state.awayPos:
#             opponents.append([Vector2D(int(i.x),int(i.y)),j])
#             j+=1
#         opponents.sort(key=lambda opp: self.Dist(opp,state))
#         opponents=opponents[0:4]
#         angles=[]
#         for opponent in opponents:
#             print "x: "+str(opponent[0].x)+" y: "+str(opponent[0].y)
#             print "my theta of :"+str(self.bot_id)+" is "+str((state.homePos[self.bot_id].theta)/pi*180)
#             angle=opponent[0].normalizeAngle(opponent[0].angle(botPos) - (state.homePos[self.bot_id].theta))
#             print "opp theta of :"+str(opponent[1])+" is "+str(angle*180/pi)
#             thresh=k/pow(self.Dist(opponent,state),0)       #threshold made constant
#             print "and the threshold is: "+str(thresh*180/pi)
#             angles.append([angle-thresh,angle+thresh,opponent[1]])
#         print "angles are: "+str(angles)
#         players=[]
#         for i in xrange(len(state.homePos)):
#             if i!=self.bot_id:
#                 players.append([Vector2D(int(state.homePos[i].x), int(state.homePos[i].y)),i])
#         players.sort(key=lambda player: self.Dist(player,state))
#         players=players[0:4]
#         myplayer=0
#         flag=False
#         for player in players:
#             isinrange=False
#             count=0
#             for angle in angles:
#               if (player[0].angle(botPos)- (state.homePos[self.bot_id].theta))<=angle[1] and (player[0].angle(botPos)- (state.homePos[self.bot_id].theta))>=angle[0]:
#                 isinrange=True
#                 homeDist=player[0].dist(botPos)
#                 awayDist=Vector2D(int(state.awayPos[angle[2]].x),int(state.awayPos[angle[2]].y)).dist(botPos)
#                 diff=awayDist - homeDist
#                 if diff > 10: flag=True; myplayer=player; break
#               count+=1
#             if isinrange==False:
#                 myplayer=player
#                 flag=True; break
#             if flag==True: break
#         FinalScore=[]
#         if myplayer==0:
#             print "No player could be determined"
#             lineangles=[]
#             i=0; j=0
#             while (1):
#                 j=i+1
#                 if j>3: j=0
#                 if i>3: break
#                 if angles[j][0]-angles[i][1]>(15*180/pi):
#                     lineangles.append((angles[i][1]+angles[j][0])/2)
#                 i+=1
#             players=[]
#             for opponent in opponents:
#                 print opponent[1]
#             for i in lineangles:
#                 print i*180/pi
#             for i in xrange(len(state.homePos)):
#                 if i!=self.bot_id:
#                     players.append([Vector2D(int(state.homePos[i].x), int(state.homePos[i].y)),i])
#             players.sort(key=lambda player: self.Dist3(player,state))
#             players=players[0:len(lineangles)]
#             corrAngles=[]
#             for player in players:
#                 Scores=[]
#                 for angle in lineangles:
#                     Scores.append([self.DistTheta(angle,state.homePos[self.bot_id].theta,player[0],botPos),botPos])
#                 Scores.sort(key=lambda score: self.Dist2(score,state))
#                 corrAngles.append(Scores[0][0])


        
#             for i in xrange(len(lineangles)):
#                 distance=self.DistTheta(corrAngles[i],state.homePos[self.bot_id].theta,players[i][0],botPos)
#                 try:
#                     FinalScore.append([fabs(k2/distance),players[i][1]])
#                 except:
#                     FinalScore.append([99999999999999999999999999,players[i][1]])
#             FinalScore.sort(key=lambda score: self.Dist2(score,state))
#         else: print "bot found :) :):) :) :) :) :) :) :) :) :) :) :) :) " + str(myplayer[1])

#         import sKickToPoint
#         if myplayer!=0:
#             self.sParam.KickToPointP.x    = myplayer[0].x
#             self.sParam.KickToPointP.y    = myplayer[0].y
#         else:
#             print FinalScore
#             self.sParam.KickToPointP.x    = state.homePos[FinalScore[0][1]].x
#             self.sParam.KickToPointP.y    = state.homePos[FinalScore[0][1]].y
#         self.sParam.KickToPointP.power    = 7
#         print self.sParam.KickToPointP.x
#         print "a"+str(self.sParam.KickToPointP.y)
#         #print FinalScore[0][1]

#         sKickToPoint.execute(self.sParam, state, self.bot_id, pub)
#         #import TMark

#         #sGoToBallOpp.execute(self.sParam,state,closest_opp,pub)
#         import sGoToPointOpp
#         self.sParam.GoToPointP.x=(self.sParam.KickToPointP.x + state.homePos[self.bot_id].x)/2
#         self.sParam.GoToPointP.y=(self.sParam.KickToPointP.y + state.homePos[self.bot_id].y)/2
#         sGoToPointOpp.execute(self.sParam,state,0,pub)


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
