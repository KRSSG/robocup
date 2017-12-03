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
from numpy import inf, array
from config import *
MAX_DRIBBLE_RANGE = 3
KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
THRES = 0.8
THETA_THRESH = 0.005
GOALIE_MAXX = 2450
GOALIE_MAXY =650

SHIFT = (HALF_FIELD_MAXX/9.0)
KICK_THRESH = 0.6
class isPossible:
    def __init__(self,botID,state):
        self.bot_id = botID
    #returns angle at vertex P1
    # def range_merge(self,List):
    #     #List = [[-1, 2], [0, 3], [2, 3], [3, 4], [4, 6]]
    #     mergedList = []
    #     #print("no of bots in side ", len(List))
    #     if len(List) is 0:
    #         return mergedList
    #     List.sort(key = lambda x:x[0])
    #     mergedList.append(List[0])
    #     t = 0
    #     for i in xrange(len(List)-1):
    #         if List[i+1][0] <= mergedList[t][1] and List[i+1][1]>=mergedList[t][1]:
    #             mergedList.append([mergedList[t][0], List[i+1][1]])
    #             mergedList.remove(mergedList[t])
    #         elif List[i+1][1] <= mergedList[t][1]:
    #             pass
    #         else:
    #             mergedList.append(List[i+1])
    #             t+=1
    #     return mergedList




    def OpenAngle(self, state, bot_id):
        def range_merge(List):
            #List = [[-1, 2], [0, 3], [2, 3], [3, 4], [4, 6]]
            mergedList = []
            #print("no of bots in side ", len(List))
            if len(List) is 0:
                return mergedList
            List.sort(key = lambda x:x[0])
            mergedList.append(List[0])
            t = 0
            for i in xrange(len(List)-1):
                if List[i+1][0] <= mergedList[t][1] and List[i+1][1]>=mergedList[t][1]:
                    mergedList.append([mergedList[t][0], List[i+1][1]])
                    mergedList.remove(mergedList[t])
                elif List[i+1][1] <= mergedList[t][1]:
                    pass
                else:
                    mergedList.append(List[i+1])
                    t+=1
            return mergedList

        #########################################################
        away_in_our_side = []
        home_in_our_side = []
        #print("yooyoyooy")
        kickerBot = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))

        Upper_limit = Vector2D(int(HALF_FIELD_MAXX), int(OUR_GOAL_MAXY))
        Lower_limit = Vector2D(int(HALF_FIELD_MAXX), int(OUR_GOAL_MINY))
        #print("chhhu",Lower_limit.y)
        self.upper_angle = atan((Upper_limit.y-state.homePos[bot_id].y)*1.0/ (Upper_limit.x-state.homePos[bot_id].x))
        self.lower_angle = atan((Lower_limit.y-state.homePos[bot_id].y)*1.0/ (Lower_limit.x-state.homePos[bot_id].x))
        #print(57.2*self.upper_angle,57.2*self.lower_angle,"chhuu")
        #our bots on our side

        home_in_our_side = [x for x in xrange(6)]
        away_in_our_side = [x for x in xrange(6)]



        angle_bot_to_kicker_list = []

        for bots in away_in_our_side:
            angle_bot_to_kicker = atan((state.homePos[bot_id].y-state.awayPos[bots].y)*1.0/ (state.homePos[bot_id].x-state.awayPos[bots].x))
            #print(bots, angle_bot_to_kicker,"gf")
            dist_of_bot = kickerBot.dist(Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y)))
            thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            if state.awayPos[bots].x > state.homePos[bot_id].x :
                if angle_bot_to_kicker - thres_theta > self.lower_angle and angle_bot_to_kicker + thres_theta < self.upper_angle:
                    angle_bot_to_kicker_list.append([angle_bot_to_kicker - thres_theta, angle_bot_to_kicker + thres_theta])
                # elif self.upper_angle - (angle_bot_to_kicker + thres_theta)  < 0.1 and (angle_bot_to_kicker - thres_theta >self.lower_angle and angle_bot_to_kicker - thres_theta < self.upper_angle):
                #     angle_bot_to_kicker_list.append([angle_bot_to_kicker + thres_theta, self.upper_angle])
                # elif angle_bot_to_kicker - thres_theta - self.lower_angle  < 0.1 and (angle_bot_to_kicker + thres_theta >self.lower_angle and angle_bot_to_kicker + thres_theta < self.upper_angle):
                #     angle_bot_to_kicker_list.append([self.lower_angle, angle_bot_to_kicker - thres_theta])


        for bots in home_in_our_side:
            if bots is bot_id:
                continue
            #print(bots,bot_id)
            if int(state.homePos[bot_id].x-state.homePos[bots].x) is 0:
                if state.homePos[bot_id].y-state.homePos[bots].y < 0:
                    angle_bot_to_kicker = -pi/ 2
                elif state.homePos[bot_id].y-state.homePos[bots].y > 0 :
                    angle_bot_to_kicker = pi/ 2
                else:
                    angle_bot_to_kicker = 0
            else:
                angle_bot_to_kicker = atan((state.homePos[bot_id].y-state.homePos[bots].y)*1.0/ (state.homePos[bot_id].x-state.homePos[bots].x))
            #print(bots, angle_bot_to_kicker,"gf")
            dist_of_bot = kickerBot.dist(Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y)))
            thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            if state.homePos[bots].x > state.homePos[bot_id].x :
                if angle_bot_to_kicker - thres_theta > self.lower_angle and angle_bot_to_kicker + thres_theta < self.upper_angle:
                    angle_bot_to_kicker_list.append([angle_bot_to_kicker - thres_theta, angle_bot_to_kicker + thres_theta])
                # elif self.upper_angle - (angle_bot_to_kicker + thres_theta)  < 0.1 and (angle_bot_to_kicker - thres_theta >self.lower_angle and angle_bot_to_kicker - thres_theta < self.upper_angle):
                #     angle_bot_to_kicker_list.append([angle_bot_to_kicker + thres_theta, self.upper_angle])
                # elif angle_bot_to_kicker - thres_theta - self.lower_angle  < 0.1 and (angle_bot_to_kicker + thres_theta >self.lower_angle and angle_bot_to_kicker + thres_theta < self.upper_angle):
                #     angle_bot_to_kicker_list.append([self.lower_angle, angle_bot_to_kicker - thres_theta])
                #print(bots,state.homePos[bots].x > state.homePos[bot_id].x)
        
        closed_angle_list = range_merge(angle_bot_to_kicker_list)
        open_angles = []
        k = 1
        if len(closed_angle_list):
            open_angles.append([self.lower_angle, closed_angle_list[0][0], closed_angle_list[0][0]-self.lower_angle])
            for i in xrange(len(closed_angle_list)-1):
                open_angles.append([closed_angle_list[i][k], closed_angle_list[i+1][1-k], closed_angle_list[i+1][1-k]-closed_angle_list[i][k]])
                k = 1-k
            open_angles.append([closed_angle_list[len(closed_angle_list)-1][1], self.upper_angle, self.upper_angle-closed_angle_list[len(closed_angle_list)-1][1]])
            open_angles.sort(key = lambda x:x[2], reverse = True)
        else:
            open_angles.append([self.lower_angle, self.upper_angle, self.upper_angle-self.lower_angle])

        #print("closed",array(closed_angle_list)*57.2)
        return open_angles
        
        # print("open",array(open_angles)*57.2)
    def determine_line_y(self,theta,Y,X,x):
        m = tan(theta) 
        if m > 65555:
            m = inf
            return 0
        return m*(x-X)+Y
    def isGoalPossible(self,state):
        kickerBot = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        open_angles = self.OpenAngle(state,self.bot_id)

        #print("open",array(open_angles)*57.2)
        #print("Goal kick score ", open_angles[0][2]/(self.upper_angle-self.lower_angle))
        score = open_angles[0][2]/(self.upper_angle-self.lower_angle)
        y1 = self.determine_line_y(open_angles[0][0], state.homePos[self.bot_id].y, state.homePos[self.bot_id].x, HALF_FIELD_MAXX)
        y2 = self.determine_line_y(open_angles[0][1], state.homePos[self.bot_id].y, state.homePos[self.bot_id].x, HALF_FIELD_MAXX)
        # print(y1,y2)
        # return False,0






        # for bots in home_in_our_side:
        #     if bot_id is bots:
        #         continue
        #     try:
        #         angle_bot_to_kicker = atan((state.homePos[self.bot_id].y-state.homePos[bots].y)*1.0/ (state.homePos[self.bot_id].x-state.homePos[bots].x))
        #     except:
        #         angle_bot_to_kicker = 0.5*pi*fabs(state.homePos[self.bot_id].y-state.homePos[bots].y)/ state.homePos[self.bot_id].y-state.homePos[bots].y 
        #     t_remove = []
        #     for t in triangle:
        #         #print"abc:   "+str(t[0])

        #         if self.self.point_in_triangle(t, Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y))):
        #             #temp_theta = asin(BOT_RADIUS/kickerBot.dist(Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y))))
        #             vertexAngle = self.angle_at_vextex(t[0], t[1], t[2])
        #             #print("vertex      ", vertexAngle)
        #             self.upper_angle = atan((t[1].y-state.homePos[self.bot_id].y)/ (t[1].x-state.homePos[self.bot_id].x))
        #             self.lower_angle = atan((t[2].y-state.homePos[self.bot_id].y)/ (t[2].x-state.homePos[self.bot_id].x)) 
        #             open_angle_to_remove = []
        #             for i in xrange(len(OpenAngle)):
        #                 #print("fuck off angle ",i,OpenAngle[i][0])
        #                 if fabs(OpenAngle[i][0] - vertexAngle) <0.000000001:
        #                     open_angle_to_remove.append(i)

        #             for i in xrange(len(open_angle_to_remove)):
        #                 OpenAngle.remove(OpenAngle[i])
        #             #print("length open angles ",len(OpenAngle))
                            

        #             m = (state.homePos[self.bot_id].y-state.homePos[bots].y)/ (state.homePos[self.bot_id].x-state.homePos[bots].x)
        #             new_point = Vector2D(int(HALF_FIELD_MAXX), int(m*(HALF_FIELD_MAXX-state.homePos[self.bot_id].x)+state.homePos[self.bot_id].y))
                    
        #             angle_1 = fabs(angle_bot_to_kicker - self.upper_angle)
        #             angle_2 = fabs(angle_bot_to_kicker - lower_angle)

        #             if angle_1 < angle_2:
        #                 OpenAngle.append([angle_1, t[1], new_point])
        #                 OpenAngle.append([angle_2, new_point, t[2]])
        #             else:
        #                 OpenAngle.append([angle_2, t[2], new_point])
        #                 OpenAngle.append([angle_1, t[1],new_point])
        #             #print("length open angles2 ",len(OpenAngle))

                    
        #             temp_1 = t[1].dist(new_point)
        #             temp_2 = t[2].dist(new_point)

        #             if temp_1<temp_2:
        #                 t_new.append([kickerBot, t[1], new_point])
        #             else:
        #                 t_new.append([kickerBot, t[2], new_point])
        #             #print("length of train ",len(triangle))
        #             t_remove.append(t)
                    
        #     for t_rem in t_remove:
        #         triangle.remove(t_rem)
        #     for t_app in t_new:
        #         triangle.append(t_app)
        #     t_remove = []    
        #     #print("length of trainangle2 ",len(triangle))
        #     t_new = []

        

        # from numpy import array
        
        # OpenAngle.sort(key = lambda x:x[0], reverse = True)
        # print("Angle sorted")
        # for i in range(len(OpenAngle)):
        #     print(OpenAngle[i][0]*180/pi)
        # # for i in xrange(len(OpenAngle)):
        # #     print("Openangle prwsent .......",180*OpenAngle[i][0]/ pi)
        # if OpenAngle[0][0] >= 3*pi/20:
        #     #print("chutiyha hai tu #jisko pta hai usko pta hai.....")
        #     # for i in xrange(len(OpenAngle)):
        #     #     print("__________OPenAngle__________",OpenAngle[i][0])

        Goal_Point = Vector2D(int(HALF_FIELD_MAXX), int(0.5*(y1+y2)))
        if score > 0.45 and score < 0.55:
            time_avil = sqrt(2.0*fabs(y2-y1)/ MAX_BOT_ACCELERATION)
        
            time_req = kickerBot.dist(Goal_Point)/ MAX_BALL_SPEED
            #print("time params",time_avil, time_req)
            if time_avil>time_req:
                return True,Goal_Point,score
            else:
                    return False,kickerBot,score
        elif score > 0.55:
            return True,Goal_Point,score
        else:
            return False,kickerBot,score










    


    def isPassPossible(self, state, receiver_bot_id):

        kickerBot = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        receiverBot = Vector2D(int(state.homePos[receiver_bot_id].x), int(state.homePos[receiver_bot_id].y))
        try:
            m = (receiverBot.y-kickerBot.y)*1.0/ (receiverBot.x-kickerBot.x)
        except:
            m = (receiverBot.y-kickerBot.y)*1.0/ (receiverBot.x-kickerBot.x+0.0000001)
        
        c = receiverBot.y - m*receiverBot.x



        theta_diff = state.homePos[self.bot_id].theta - state.homePos[receiver_bot_id].theta
        time_req_turn_intercept = fabs(theta_diff*2.0/MAX_BOT_SPEED) + fabs((kickerBot.dist(receiverBot))*1.0/MOVING_BALL_VELOCITY)+KICK_THRESH
        time_req_turn_intercept_opp = []

        for away_bot in xrange(len(state.awayPos)):
            dist_from_line = fabs(state.awayPos[away_bot].y-m*state.awayPos[away_bot].x-c)*1.0/ sqrt(m**2+1)
            time_req_turn_intercept_opp.append([away_bot, dist_from_line/ MAX_BOT_SPEED])
        
        time_req_turn_intercept_opp.sort(key=lambda x:x[1])

        if time_req_turn_intercept < time_req_turn_intercept_opp[0]:
            return True
        else:
            return False

    def isReceivePossible(self, state, receiver_bot_id):
        kickerBot = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        receiverBot = Vector2D(int(state.homePos[receiver_bot_id].x), int(state.homePos[receiver_bot_id].y))
        
        try:
            angle = atan((receiverBot.y-kickerBot.y)*1.0/ (receiverBot.x-kickerBot.x))
        except:
            angle = pi/2

        angle_diff = fabs(state.homePos[receiver_bot_id].theta - angle)

        dist_from_kicker = kickerBot.dist(receiverBot)

        time_avaib = dist_from_kicker*1.0/ MAX_BOT_SPEED
        time_req = angle_diff*1.0/ MOVING_BALL_VELOCITY

        if time_avaib>time_req:
            return True
        else:
            return False











