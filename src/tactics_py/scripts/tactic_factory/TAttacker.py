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

class TAttacker(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TAttacker, self).__init__( bot_id, state, param)
        self.oh_my_bot =-1
        self.bot_id = bot_id
        self.sParams = skills_union.SParam()
        self.UPPER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MAXY)
        self.LOWER_HALF = Vector2D(-HALF_FIELD_MAXX,OUR_GOAL_MINY)

        self.GOAL_UPPER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MAXY*3)
        self.GOAL_LOWER = Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY*3)


    def Dist(self,a,state):
        a=a[0]
        return sqrt(pow((a.x-state.homePos[self.bot_id].x),2)+ pow((a.y-(state.homePos[self.bot_id].y)),2))
    

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
        # print(opponents)
        angles=[]
        for opponent in opponents:
            ##print "x: "+str(opponent[0].x)+" y: "+str(opponent[0].y)
            ##print "my theta of :"+str(self.bot_id)+" is "+str((state.homePos[self.bot_id].theta)/pi*180)
            angle=opponent[0].normalizeAngle(opponent[0].angle(botPos) - (state.homePos[self.bot_id].theta))
            ##print "opp theta of :"+str(opponent[1])+" is "+str(angle*180/pi)
            angles.append([angle,opponent[1]])
        angles.sort(key=lambda x: x[0])
        ##print "angles are: "+str(angles)
        self.Opp_sorted_Angles=[]
        for angle in angles:
            self.Opp_sorted_Angles.append([angle[0]-THRESH,angle[0]+THRESH,angle[1]])
        lineangles=[]
        angle_gap=18.314*pi/180
        angle_to_shoot=[]
        i=0; j=0
        weights=[]
        ##print "Opp angles: "+str(self.Opp_sorted_Angles)


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
                ##print "angle here inside: "+str(array(angle_here)*(180/pi))
                if angle_here>pi:
                    angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
                else:
                    angle_to_shoot.append(origin_angle+angle_here/2) 

        ##print("Angle to shoot "+ str(array(angle_to_shoot)*(180/pi)))

        for i in xrange(len(state.homePos)):
            if i==self.bot_id: continue
            dist=fabs(Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            angle_here=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)).angle(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))) - state.homePos[self.bot_id].theta
            min_dist= np.inf
            # #print "angles to shoot "+str(angle_to_shoot)
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
        # #print "goal angle lower : "+str(goal_angle_lower*180/pi)
        # #print "goal angle upper: "+str(goal_angle_upper*180/pi)     
        # #goal_angle_lower = atan2(LOWER_HALF.y-state.homePos[self.bot_id].y, LOWER_HALF.x-state.homePos[self.bot_id].x)
        for i in xrange(len(state.homePos)):
            if i==self.bot_id: continue
            myBotPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            angle=myBotPos.angle(botPos)-state.homePos[self.bot_id].theta
            ourGoal=Vector2D(int(-HALF_FIELD_MAXX),int(0))
            ##print "angle for bot "+str(i)+" is "+str(angle*180/pi)
            flag1 =0
            for angles in self.Opp_sorted_Angles:
                if (angle>=angles[0] and angle<=angles[1])  :

                    dist_of_homeBot=myBotPos.dist(botPos)
                    dist_of_awayBot=Vector2D(int(state.awayPos[angles[2]].x),int(state.awayPos[angles[2]].y)).dist(botPos)
                   
                    if fabs(dist_of_awayBot)<fabs(dist_of_homeBot)+BOT_OPP_DIST_THRESH:
                        self.bots_to_remove.append(i)
                        flag1 =1
            distance1 = ourGoal.dist(myBotPos)
            # print("calculation",i,distance1,flag1,-HALF_FIELD_MAXX/2)
            if distance1 < HALF_FIELD_MAXX/2 and flag1==0:
                self.bots_to_remove.append(i)
                

        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        try:
            goal_angle_upper = atan((self.UPPER_HALF.y-botPos.y)*1.0/ (self.UPPER_HALF.x-botPos.x))
        except:
            goal_angle_upper = (fabs(self.UPPER_HALF.y-botPos.y)/(self.UPPER_HALF.y-botPos.y))*pi/2
        try:
            goal_angle_lower = atan((self.LOWER_HALF.y-botPos.y)*1.0/ (self.LOWER_HALF.x-botPos.x))
        except:
            goal_angle_lower = (fabs(self.LOWER_HALF.y-botPos.y)/(self.UPPER_HALF.y-botPos.y))*pi/2

        ##print("signs ",(self.UPPER_HALF.y-botPos.y)* (self.UPPER_HALF.x-botPos.x), (self.LOWER_HALF.y-botPos.y)* (self.LOWER_HALF.x-botPos.x))
        #goal_angle_upper,goal_angle_lower = map(botPos.normalizeAngle, (goal_angle_upper,goal_angle_lower))
        ##print(goal_angle_lower,goal_angle_upper,"fhasdkfasdk")
        
        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            if our_bot  in self.bots_to_remove:
                continue 
            bot_angle = atan((state.homePos[our_bot].y - botPos.y)/ (state.homePos[our_bot].x - botPos.x))
            ##print("bot angle", bot_angle ,our_bot)
            # flag=-1
            if state.isteamyellow==0:
                if bot_angle > goal_angle_upper and bot_angle < goal_angle_lower and state.homePos[our_bot].x < botPos.x :
                    self.bots_to_remove.append(our_bot)
            else:
                if bot_angle > goal_angle_upper and bot_angle < goal_angle_lower and state.homePos[our_bot].x > botPos.x :
                    self.bots_to_remove.append(our_bot)

        ##print("bots to remove            ", self.bots_to_remove)
        # print("printing weights")
        # for i in xrange(len(weights)):
        #     print weights[i]

        # print("printing bots to remove")
        # for i in xrange(len(self.bots_to_remove)):
        #     print self.bots_to_remove[i]
        if state.our_goalie in self.bots_to_remove:
            pass
        else:
            self.bots_to_remove.append(state.our_goalie)
        # print("printing bots to remove")
        # for i in xrange(len(self.bots_to_remove)):
            # print self.bots_to_remove[i]
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

    def bot_ourgoal_angle(self,state):
        angles=[]
        botPos = Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))
        Goal = Vector2D(int(-HALF_FIELD_MAXY),int(0))
        for i in xrange(len(state.homePos)):
            if i is self.bot_id:
                continue
            anglediff = fabs(botPos.angle(Goal) - botPos.angle(Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))))
            angles.append([i,anglediff])
        return angles

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
            #if distance is greater than optimal distance then score will be greater than 1.... ERROR
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
        ##print(ourBot_app_wt)
        ##print("bod_id  ",self.bot_id)
        return ourBot_app_wt

    
                

    def calculate_score(self, state,BOTiD):

        weights = [0.35, 0.25, 0.15, 0.35, 0.15,0.6]
        dist_weight_partition = [0.25, 0.75]

        score_bisect   = self.bisector_bot_wt(state)
        score_orient   = self.bot_orient_wt(state)
        #score_velo_dir = self.velocity_dir_wt(state)
        score_opt_dist = self.optimal_dist_wt(state)
        score_bot_app  = self.ourBot_approachability_wt(state)
        score_bot_goalangle = self.bot_ourgoal_angle(state)


        # #print "score opt distance: "+str(score_opt_dist[1])

        score = []
    
        our_bot=0
        for i in xrange(len(state.homePos)):
            if i is BOTiD :
               
                continue
           
            botid = score_bisect[our_bot][0]
            bot_score = (score_bisect[our_bot][1]*weights[0]*dist_weight_partition[0] + score_bisect[our_bot][2]*weights[0]*dist_weight_partition[1]) +\
            (score_orient[our_bot][1]*weights[1])+\
            (score_opt_dist[our_bot][1]*weights[3])+\
            (score_bot_app[our_bot][1]*weights[4])


            if botid in self.bots_to_remove:
                score.append([botid,0])
            else:
                score.append([botid, bot_score])
            our_bot+=1
        score.sort(reverse= True ,key = lambda x:x[1] )
      
        return score
    

    def goal_possible(self,state):

        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))

        goal_angle_upper = atan((self.GOAL_UPPER.y-botPos.y)*1.0/ (self.GOAL_UPPER.x-botPos.x))
        goal_angle_lower = atan((self.GOAL_LOWER.y-botPos.y)*1.0/ (self.GOAL_LOWER.x-botPos.x))
        goal_angle_mid   = 0.5*(goal_angle_lower+goal_angle_upper)

        
        k1 = False
        k2 = False
        for away_bot in xrange(len(state.awayPos)):
            
            if state.awayPos[away_bot].x > state.homePos[self.bot_id].x:
                ##print (state.awayPos[away_bot].x, state.homePos[self.bot_id].x, "chutiya.....")
                away_BOT = Vector2D (int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y))
                try:
                    bot_angle = atan((away_BOT.y - botPos.y)*1.0/ (away_BOT.x - botPos.x))
                except:
                    bot_angle = fabs((away_BOT.y - botPos.y))

                ##print("bhosdiwala angles ",bot_angle, goal_angle_lower, goal_angle_mid, goal_angle_upper, away_bot)
                if bot_angle > goal_angle_lower and bot_angle < goal_angle_mid:
                    k1=True
                     
                if bot_angle > goal_angle_mid and bot_angle < goal_angle_upper:
                    k2=True
                #print(k1,k2)
        for home_bot in xrange(len(state.homePos)):
            
            if state.homePos[home_bot].x > state.homePos[self.bot_id].x:
                ##print (state.homePos[home_bot].x, state.homePos[self.bot_id].x, "chutiya.....")
                home_BOT = Vector2D (int(state.homePos[home_bot].x), int(state.homePos[home_bot].y))
                try:
                    bot_angle = atan((home_BOT.y - botPos.y)*1.0/ (home_BOT.x - botPos.x))
                except:
                    bot_angle = fabs((home_BOT.y - botPos.y))

                ##print("bhosdiwala angles ",bot_angle, goal_angle_lower, goal_angle_mid, goal_angle_upper, home_bot)
                if bot_angle > goal_angle_lower and bot_angle < goal_angle_mid:
                    k1=True
                     
                if bot_angle > goal_angle_mid and bot_angle < goal_angle_upper:
                    k2=True
                    
        if k1 and k2:
            return [False, -1]
        if k1 :
            return [True, 2]
        if k2 :
            return [True, 1]
        if not (k1 or k2):
            return [True, 0]


    def execute(self, state, pub, receiver_bot_id):
        print(" in attacker ", self.bot_id)
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        DistancesFromBot = inf

        
        import sKickToPoint
        import sGoToBall

        if not (state.ballPos.x < -HALF_FIELD_MAXX + DBOX_WIDTH  and  fabs(state.ballPos.y) < OUR_GOAL_MAXY*0.8) :
            # print(ballPos.x,ballPos.y)
            if fabs(ballPos.dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y)))>BOT_BALL_THRESH):
                sGoToBall.execute(self.sParams, state, self.bot_id, pub)
        else:
            return
        flaggu=0
        possibility = isPossible(self.bot_id,state)
        bool_goal = possibility.isGoalPossible(state)
        goal_chck = self.goal_possible(state)




        scores_of_bot = self.calculate_score(state,self.bot_id)
        AttackBot_ID = self.bot_id
        
        self.oh_my_bot = scores_of_bot[0][0]
        self.oh_my_bot =1
        #print("\n\n\n\n\n\n\n\n\n\nScore: "+str(scores_of_bot))
        # print("kickinf to",self.oh_my_bot)
        passprobable = possibility.isPassPossible(state, self.oh_my_bot)
        geom = Vector2D()
        angle2 = geom.normalizeAngle(passprobable[1])
        receiveprobable = possibility.isReceivePossible(state, self.oh_my_bot,angle2)
        print ("_PASS_PROBABLE_",passprobable[0])
        if not flaggu :    
            if not passprobable[0]:
                print("Passing",self.bot_id,self.oh_my_bot)
                if receiveprobable:
                    print("Receiving",self.bot_id,self.oh_my_bot)
                    receiverbot = Vector2D(int(state.homePos[self.oh_my_bot].x),int(state.homePos[self.oh_my_bot].y))
                    receiver_bot_id[0] = self.oh_my_bot
                    
                    
                    distance = ballPos.dist(receiverbot)

                    point =Vector2D(int(ballPos.x + distance*cos(angle2)),int(ballPos.y + distance*sin(angle2)))

                    # print(self.oh_my_bot)
                    # ob =Vector2D()
                    # finalSlope = ballPos.angle(Vector2D(int(state.homePos[self.oh_my_bot].x),int(state.homePos[self.oh_my_bot].y)))
                    # turnAngleLeft = ob.normalizeAngle(finalSlope - state.homePos[self.bot_id].theta)
                    botPos=Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
                    # ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))
                    dist = ballPos.dist(botPos)
                    # finalSlope1 = ballPos.angle(botPosition)
                    # anglediff = state.homePos[self.bot_id].theta - ballPos.angle(Vector2D(botPosition))
                    # turnAngleLeft1 = ob.normalizeAngle(finalSlope1 - state.homePos[self.bot_id].theta) 
                    if dist > BOT_BALL_THRESH :

                        sGoToBall.execute(self.sParams, state,self.bot_id, pub)
                        return 
                    # elif math.fabs(turnAngleLeft1) > SATISFIABLE_THETA/2 : # SATISFIABLE_THETA in config file
                    #     sParam = skills_union.SParam()
                    #     import sTurnToPoint
                    #     sParam.TurnToPointP.x = botPosition.x
                    #     sParam.TurnToPointP.y = botPosition.y
                    #     sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA
                    #     print("before turn")
                    #     sTurnToPoint.execute(self.sParams, state,self.bot_id, pub)
                    #     print("after turn")
                    #     return
                    # elif fabs(anglediff)>SATISFIABLE_THETA:
                    #     argument = "align_properly"
                    #     print(argument)
                    #     return
                    else:
                        # self.sParams.KickToPointP.x = state.homePos[self.oh_my_bot].x       
                        # self.sParams.KickToPointP.y = state.homePos[self.oh_my_bot].y
                        self.sParams.KickToPointP.x = point.x       
                        self.sParams.KickToPointP.y = point.y
                        # print("cordadfsd",state.homePos[self.oh_my_bot].x,state.homePos[self.oh_my_bot].y)
                        self.sParams.KickToPointP.power         = 7
                        sKickToPoint.execute(self.sParams, state, self.bot_id, pub)
                        # argument = "kicking"
                        # print(argument)
                        return
            else:
                print("______________kick to goal___________")
                self.sParams.KickToPointP.x = 3000       
                self.sParams.KickToPointP.y = 0
                # print("cordadfsd",state.homePos[self.oh_my_bot].x,state.homePos[self.oh_my_bot].y)
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


