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


KICK_RANGE_THRESH = MAX_DRIBBLE_R   #ASK
THRES  = 0.8
THETA_THRESH = 0.005
TURNING_THRESH = 10
# k=k2=50000 no threshold
THRESH=15*pi/180

ROWS                                    =  33
COLS                                    =  22
OPTIMAL_DISTANCE                        =  3.0*HALF_FIELD_MAXX/5
BISECTOR_CONST                          =  40
OUR_BOT_APPROACHABILITY_CONST           =  25 
OPTIMAL_DISTANCE_CONST                  =  10

k=k2=20*pi/180

class TPosition(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()


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
        lineangles=[]
        angle_gap=18.314*pi/180
        angle_to_shoot=[]
        i=0; j=0
        weights=[]
        #print "Opp angles: "+str(self.Opp_sorted_Angles)
        for i in xrange(len(self.Opp_sorted_Angles)):
            origin_angle=self.Opp_sorted_Angles[i][1]
            j=i+1
            if j>=len(self.Opp_sorted_Angles): j=0
            if self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]>angle_gap:
                angle_here=self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]
                if angle_here>pi:
                    angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
                else:
                    angle_to_shoot.append(origin_angle+angle_here/2)
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
        return weights
    def bot_orient_wt(self,state):
        orient_angles= []
        # deviation=[]
        no_use = Vector2D()
        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            
            # deviation.append([our_bot, normalizeDistance])
            orient_angles.append([our_bot, no_use.normalizeAngle(state.homePos[our_bot].theta-state.homePos[self.bot_id].theta)])

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
    # def opponent_approachability_wt(self, state):

    #     opp_app_wt = []
    #     opp_dist = []

    #     for opp_id in xrange(len(state.awayPos)):
    #         distance = fabs(Vector2D(int(state.awayPos[opp_id].x), int(state.awayPos[opp_id].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))

    def ourBot_approachability_wt(self, state):
        ourBot_app_wt = []

        for our_bot in xrange(len(state.homePos)):
            if our_bot is self.bot_id:
                continue
            distance = fabs(Vector2D(int(state.homePos[our_bot].x),int(state.homePos[our_bot].y)).dist(Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))))
            normalizeDistance = OUR_BOT_APPROACHABILITY_CONST * exp(-1*distance/HALF_FIELD_MAXY)
            ourBot_app_wt.append([our_bot, normalizeDistance])
        return ourBot_app_wt

    def calculate_score(self, state):

        weights = [0.35, 0.25, 0.15, 0.05, 0.15]
        dist_weight_partition = [0.25, 0.75]

        score_bisect   = self.bisector_bot_wt(state)
        score_orient   = self.bot_orient_wt(state)
        score_velo_dir = self.velocity_dir_wt(state)
        score_opt_dist = self.optimal_dist_wt(state)
        score_bot_app  = self.ourBot_approachability_wt(state)

        # print "score opt distance: "+str(score_opt_dist[1])

        score = []
        our_bot=0
        for _ in xrange(len(state.homePos)):
            if _ is self.bot_id:
                continue
            #print "score here: "+str(score_bot_app[our_bot])
            bot_score = (score_bisect[our_bot][1]*weights[0]*dist_weight_partition[0] + score_bisect[our_bot][2]*weights[0]*dist_weight_partition[1]) +\
            (score_orient[our_bot][1]*weights[1])+\
            (score_velo_dir[our_bot][1]*weights[2])+\
            (score_opt_dist[our_bot][1]*weights[3])+\
            (score_bot_app[our_bot][1]*weights[4])
            
            score.append([our_bot, bot_score])
            our_bot+=1
        score.sort(key = lambda x:x[1])
        return score




















    


    def execute(self, state, pub):
        score=np.zeros((ROWS,COLS),dtype=np.float)
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        DistancesFromBot=999999999999
        count=0
        for i in state.homePos:
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(ballPos)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                self.bot_id=count
            count+=1
        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
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
        opponents=opponents[0:4]
        angles=[]
        for opponent in opponents:
            print "x: "+str(opponent[0].x)+" y: "+str(opponent[0].y)
            print "my theta of :"+str(self.bot_id)+" is "+str((state.homePos[self.bot_id].theta)/pi*180)
            angle=opponent[0].normalizeAngle(opponent[0].angle(botPos) - (state.homePos[self.bot_id].theta))
            print "opp theta of :"+str(opponent[1])+" is "+str(angle*180/pi)
            angles.append([angle,opponent[1]])
        angles.sort(key=lambda x: x[0])
        print "angles are: "+str(angles)
        self.Opp_sorted_Angles=[]
        for angle in angles:
            self.Opp_sorted_Angles.append([angle[0]-THRESH,angle[0]+THRESH,angle[1]])




        scores_of_bot = self.calculate_score(state)

        print("\n\n\n\n\n\n\n\n\n\nScore: "+str(scores_of_bot))





        









        # while (1):
        #     j=i+1
        #     if j>3: j=0
        #     if i>3: break
        #     if angles[j][0]-angles[i][1]>(15*180/pi):
        #         lineangles.append((angles[i][1]+angles[j][0])/2)
        #     i+=1
        # players=[]
        # for opponent in opponents:
        #     print opponent[1]
        # for i in lineangles:
        #     print i*180/pi
        # for i in xrange(len(state.homePos)):
        #     if i!=self.bot_id:
        #         players.append([Vector2D(int(state.homePos[i].x), int(state.homePos[i].y)),i])
        # players.sort(key=lambda player: self.Dist3(player,state))
        # players=players[0:len(lineangles)]
        # corrAngles=[]
        # for player in players:
        #     Scores=[]
        #     for angle in lineangles:
        #         Scores.append([self.DistTheta(angle,state.homePos[self.bot_id].theta,player[0],botPos),botPos])
        #     Scores.sort(key=lambda score: self.Dist2(score,state))
        #     corrAngles.append(Scores[0][0])



        # for i in xrange(len(lineangles)):
        #     distance=self.DistTheta(corrAngles[i],state.homePos[self.bot_id].theta,players[i][0],botPos)
        #     try:
        #         FinalScore.append([fabs(k2/distance),players[i][1]])
        #     except:
        #         FinalScore.append([np.inf,players[i][1]])
        # FinalScore.sort(key=lambda score: self.Dist2(score,state))





        # for i in xrange(len(state.homePos)):
        #     if i!=self.bot_id:
        #         players.append([Vector2D(int(state.homePos[i].x), int(state.homePos[i].y)),i])
        # players.sort(key=lambda player: self.Dist(player,state))
        # players=players[0:4]
        # myplayer=0
        # flag=False
        # for player in players:
        #     isinrange=False
        #     count=0
        #     for angle in angles:
        #       if (player[0].angle(botPos)- (state.homePos[self.bot_id].theta))<=angle[1] and (player[0].angle(botPos)- (state.homePos[self.bot_id].theta))>=angle[0]:
        #         isinrange=True
        #         homeDist=player[0].dist(botPos)
        #         awayDist=Vector2D(int(state.awayPos[angle[2]].x),int(state.awayPos[angle[2]].y)).dist(botPos)
        #         diff=awayDist - homeDist
        #         if diff > 10: flag=True; myplayer=player; break
        #       count+=1
        #     if isinrange==False:
        #         myplayer=player
        #         flag=True; break
        #     if flag==True: break
        # FinalScore=[]
        # if myplayer==0:
        #     print "No player could be determined"
        #     lineangles=[]
        #     i=0; j=0
        #     while (1):
        #         j=i+1
        #         if j>3: j=0
        #         if i>3: break
        #         if angles[j][0]-angles[i][1]>(15*180/pi):
        #             lineangles.append((angles[i][1]+angles[j][0])/2)
        #         i+=1
        #     players=[]
        #     for opponent in opponents:
        #         print opponent[1]
        #     for i in lineangles:
        #         print i*180/pi
        #     for i in xrange(len(state.homePos)):
        #         if i!=self.bot_id:
        #             players.append([Vector2D(int(state.homePos[i].x), int(state.homePos[i].y)),i])
        #     players.sort(key=lambda player: self.Dist3(player,state))
        #     players=players[0:len(lineangles)]
        #     corrAngles=[]
        #     for player in players:
        #         Scores=[]
        #         for angle in lineangles:
        #             Scores.append([self.DistTheta(angle,state.homePos[self.bot_id].theta,player[0],botPos),botPos])
        #         Scores.sort(key=lambda score: self.Dist2(score,state))
        #         corrAngles.append(Scores[0][0])


        
        #     for i in xrange(len(lineangles)):
        #         distance=self.DistTheta(corrAngles[i],state.homePos[self.bot_id].theta,players[i][0],botPos)
        #         try:
        #             FinalScore.append([fabs(k2/distance),players[i][1]])
        #         except:
        #             FinalScore.append([99999999999999999999999999,players[i][1]])
        #     FinalScore.sort(key=lambda score: self.Dist2(score,state))
        # else: print "bot found :) :):) :) :) :) :) :) :) :) :) :) :) :) " + str(myplayer[1])

        # import sKickToPoint
        # if myplayer!=0:
        #     self.sParam.KickToPointP.x    = myplayer[0].x
        #     self.sParam.KickToPointP.y    = myplayer[0].y
        # else:
        #     print FinalScore
        #     self.sParam.KickToPointP.x    = state.homePos[FinalScore[0][1]].x
        #     self.sParam.KickToPointP.y    = state.homePos[FinalScore[0][1]].y
        # self.sParam.KickToPointP.power    = 7
        # print self.sParam.KickToPointP.x
        # print "a"+str(self.sParam.KickToPointP.y)
        # #print FinalScore[0][1]

        # sKickToPoint.execute(self.sParam, state, self.bot_id, pub)
        # #import TMark

        # #sGoToBallOpp.execute(self.sParam,state,closest_opp,pub)
        # import sGoToPointOpp
        # self.sParam.GoToPointP.x=(self.sParam.KickToPointP.x + state.homePos[self.bot_id].x)/2
        # self.sParam.GoToPointP.y=(self.sParam.KickToPointP.y + state.homePos[self.bot_id].y)/2
        # sGoToPointOpp.execute(self.sParam,state,0,pub)


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


#     def isComplete(self, state):
#         # TO DO use threshold distance instead of actual co ordinates
#         if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
#             return True
#         elif time.time()-self.begin_time > self.time_out:
#             return True
#         else:
#             return False

#     def updateParams(self, state):
#         # No parameter to update here
#         pass
