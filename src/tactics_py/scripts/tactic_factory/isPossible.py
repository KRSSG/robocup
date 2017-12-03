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
GOALIE_MAXY = 400

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

    def angle_at_vextex(self,P1,P2,P3):
        a     = P2.dist(P3)
        b     = P1.dist(P3)
        c     = P1.dist(P2)
        value = (b**2 + c**2 - a**2)/(2*b*c)
        #print("acos value         ",value,a,b,c)
        theta = acos(value)
        return atan(tan(theta))

    #returns whether a pt lies inside a triangle
    def point_in_traingle(self,t, P):
        P1, P2, P3 = t[0], t[1], t[2]
        def area(p1,p2,p3):
            return abs(p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y))/2.0

        a  = area(P1,P2,P3)
        a1 = area(P,P2,P3)
        a2 = area(P1,P,P3)
        a3 = area(P1,P2,P)

        return a == a1+a2+a3


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
            try:
                angle_bot_to_kicker = atan((state.homePos[bot_id].y-state.awayPos[bots].y)*1.0/ (state.homePos[bot_id].x-state.awayPos[bots].x))
            except:
                continue
            #print(bots, angle_bot_to_kicker,"gf")
            dist_of_bot = kickerBot.dist(Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y)))
            try:
                thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            except:
                continue
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
            try:
                thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            except:
                continue
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
        distkickreceiver = kickerBot.dist(receiverBot)
        ballPos = Vector2D(int(state.ballPos.x),int(state.ballPos.y))

        middle_angle = kickerBot.angle(receiverBot)
        try:
            anglediff = asin(4*BOT_RADIUS/kickerBot.dist(receiverBot))
        except:
            anglediff = pi/ 2
        geom = Vector2D()

        upperangle = middle_angle + anglediff
        lowerangle = middle_angle - anglediff

        


        if upperangle<0:
            upperangle+=2*pi
            lowerangle+=2*pi
        if lowerangle<0:
            lowerangle+=2*pi
            upperangle+=2*pi


        # print("middle angle",middle_angle)
        # print("upper angle",upperangle)
        # print("lower angle",lowerangle)

        angles = []


        awayinregion =[]
        for awayBot in xrange(len(state.awayPos)):
            if kickerBot.dist(state.awayPos[awayBot]) < distkickreceiver:
                angle = kickerBot.angle(state.awayPos[awayBot])
                thres = asin(BOT_RADIUS/kickerBot.dist(state.awayPos[awayBot]))
                upper =angle + thres
                lower =angle - thres
                if upper < 0:
                    upper +=2*pi
                    lower +=2*pi
                if lower < 0:
                    lower +=2*pi
                    upper +=2*pi
                if (upper<upperangle and upper > lowerangle) or (lower<upperangle and lower > upperangle):
                    angles.append([lower,upper])
                    awayinregion.append(awayBot)
                #     for i in xrange(len(angles)):
                #         if upper<angles[i][1]:
                #             if lower>angles[i+1][0]:
                #                 pass
                #             continue
                #         else:
                #             if upper<angles[i][0]:
                #                 pass
                            # elif upper > angles[i][0]:
                            #     angles[i][0] = upper
        angles.sort(key=lambda x:x[0])

        # print("angles before merge",angles)

        for i in xrange(len(angles)):
            if i+1<len(angles):
                if angles[i][1] > angles[i+1][0]:
                    if angles[i][1] < angles[i+1][1]:
                        angles[i][1] = angles[i+1][1]

                    del angles[i+1]

        # print("angles after merge",angles)
        openangle=[]
        # print("length of dict angles",len(angles))


        if len(angles)==0:
            openangle.append([lowerangle,upperangle])
            return True, (upperangle + lowerangle)/2

        if angles[0][0] < lowerangle:
            angles[0][0] =lowerangle

        if angles[len(angles)-1][1] > upperangle:
            angles[len(angles)-1][1] = upperangle

        for i in xrange(len(angles)):
            if i==0:
                openangle.append([lowerangle,angles[i][0]])
            if i==len(angles)-1:
                openangle.append([angles[i][1],upperangle])
            elif i !=0 and i!= len(angles)-1:
                openangle.append([angles[i-1][1],angles[i][0]])
        # print("openangles",openangle)
        difference =[]

        for i in xrange(len(openangle)):
            diff = openangle[i][1] - openangle[i][0]
            difference.append([diff,i])
        # print("away bot in our region",awayinregion)
        difference.sort(key=lambda x:x[0],reverse=True)
        print(difference)
        for i in xrange(len(difference)):
            r = DRIBBLER_BALL_THRESH
            bestangle = (openangle[difference[i][1]][0]+openangle[difference[i][1]][1])/2 
            probable_bot_position = Vector2D(int(ballPos.x + r*cos(bestangle)),int(ballPos.y + r*sin(bestangle)))

            for i in awayinregion:
                
                k = (state.ballPos.y - probable_bot_position.y)*(state.awayPos[i].x - probable_bot_position.x)
                k -= (state.ballPos.x - probable_bot_position.x)*(state.awayPos[i].y - probable_bot_position.y)

                k /= (pow(state.ballPos.y  - probable_bot_position.y ,2) + pow(state.ballPos.x - probable_bot_position.x ,2))

                x = state.awayPos[i].x - k*(state.ballPos.y - probable_bot_position.y)
                y = state.awayPos[i].y + k*(state.ballPos.x - probable_bot_position.x)

                position =Vector2D(int(state.awayPos[i].x),int(state.awayPos[i].y))
                prep = position.dist(Vector2D(int(x),int(y)))

                if prep/MAX_BOT_SPEED > ballPos.dist(Vector2D(int(x),int(y)))/MOVING_BALL_VELOCITY or 1:
                    ############################################################################

                        # ADD HOMEBOT AND AWAYBOT VELOCITY IN BeliefState
                        # CORRECT CONDITION


                    #############################################################################
                    # print("time comparision",prep/MAX_BOT_SPEED,ballPos.dist(Vector2D(int(x),int(y)))/MOVING_BALL_VELOCITY)
                    return True,bestangle
        # difference[0][1]                    
        return False,(openangle[difference[0][1]][0] + openangle[difference[0][1]][1])/2



        # try:
        #     m = (receiverBot.y-kickerBot.y)*1.0/ (receiverBot.x-kickerBot.x)
        # except:
        #     m = (receiverBot.y-kickerBot.y)*1.0/ (receiverBot.x-kickerBot.x+0.0000001)
        
        # c = receiverBot.y - m*receiverBot.x



        # theta_diff = state.homePos[self.bot_id].theta - state.homePos[receiver_bot_id].theta
        # time_req_turn_intercept = fabs(theta_diff*2.0/MAX_BOT_SPEED) + fabs((kickerBot.dist(receiverBot))*1.0/MOVING_BALL_VELOCITY)+KICK_THRESH
        # time_req_turn_intercept_opp = []

        # for away_bot in xrange(len(state.awayPos)):
        #     dist_from_line = fabs(state.awayPos[away_bot].y-m*state.awayPos[away_bot].x-c)/((m**2+1)**0.5)
        #     time_req_turn_intercept_opp.append([away_bot, dist_from_line/MAX_BOT_SPEED])
        
        # time_req_turn_intercept_opp.sort(key=lambda x:x[1])

        # if time_req_turn_intercept < time_req_turn_intercept_opp[0]:
        #     return True
        # else:
        #     return False

    def isReceivePossible(self, state, receiver_bot_id,angle = 0):

        geom = Vector2D()

        anglediff = geom.normalizeAngle((angle + pi)-state.homePos[receiver_bot_id].theta)

        receiverPos = Vector2D(int(state.homePos[receiver_bot_id].x),int(state.homePos[receiver_bot_id].y))
        ballPos = Vector2D(int(state.ballPos.x),int(state.ballPos.y))

        ballbotdist = ballPos.dist(receiverPos)

        if fabs(anglediff)/MAX_BOT_OMEGA < ballbotdist/MOVING_BALL_VELOCITY:
            return True
        else:
            return False
    




    def GoalScore(self, state, X, Y, bot_id = -1):
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
        kickerBot = Vector2D(int(X), int(Y))

        Upper_limit = Vector2D(int(HALF_FIELD_MAXX), int(OUR_GOAL_MAXY))
        Lower_limit = Vector2D(int(HALF_FIELD_MAXX), int(OUR_GOAL_MINY))
        #print("chhhu",Lower_limit.y)
        upper_angle = atan((Upper_limit.y-Y)*1.0/ (Upper_limit.x-X))
        lower_angle = atan((Lower_limit.y-Y)*1.0/ (Lower_limit.x-X))
        #print(57.2*upper_angle,57.2*lower_angle,"chhuu")
        #our bots on our side

        home_in_our_side = [x for x in xrange(6)]
        away_in_our_side = [x for x in xrange(6)]



        angle_bot_to_kicker_list = []

        for bots in away_in_our_side:
            angle_bot_to_kicker = atan((Y-state.awayPos[bots].y)*1.0/ (X-state.awayPos[bots].x))
            #print(bots, angle_bot_to_kicker,"gf")
            dist_of_bot = kickerBot.dist(Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y)))
            try:
                thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            except:
                continue
            if state.awayPos[bots].x > X :
                if angle_bot_to_kicker - thres_theta > lower_angle and angle_bot_to_kicker + thres_theta < upper_angle:
                    angle_bot_to_kicker_list.append([angle_bot_to_kicker - thres_theta, angle_bot_to_kicker + thres_theta])
                # elif upper_angle - (angle_bot_to_kicker + thres_theta)  < 0.1 and (angle_bot_to_kicker - thres_theta >lower_angle and angle_bot_to_kicker - thres_theta < upper_angle):
                #     angle_bot_to_kicker_list.append([angle_bot_to_kicker + thres_theta, upper_angle])
                # elif angle_bot_to_kicker - thres_theta - lower_angle  < 0.1 and (angle_bot_to_kicker + thres_theta >lower_angle and angle_bot_to_kicker + thres_theta < upper_angle):
                #     angle_bot_to_kicker_list.append([lower_angle, angle_bot_to_kicker - thres_theta])


        for bots in home_in_our_side:
            if bots is bot_id:
                continue
            #print(bots,bot_id)
            if int(X-state.homePos[bots].x) is 0:
                if Y-state.homePos[bots].y < 0:
                    angle_bot_to_kicker = -pi/ 2
                elif Y-state.homePos[bots].y > 0 :
                    angle_bot_to_kicker = pi/ 2
                else:
                    angle_bot_to_kicker = 0
            else:
                angle_bot_to_kicker = atan((Y-state.homePos[bots].y)*1.0/ (X-state.homePos[bots].x))

            dist_of_bot = kickerBot.dist(Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y)))
            try:
                thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            except:
                continue
            if state.homePos[bots].x > X :
                if angle_bot_to_kicker - thres_theta > lower_angle and angle_bot_to_kicker + thres_theta < upper_angle:
                    angle_bot_to_kicker_list.append([angle_bot_to_kicker - thres_theta, angle_bot_to_kicker + thres_theta])
                # elif upper_angle - (angle_bot_to_kicker + thres_theta)  < 0.1 and (angle_bot_to_kicker - thres_theta >lower_angle and angle_bot_to_kicker - thres_theta < upper_angle):
                #     angle_bot_to_kicker_list.append([angle_bot_to_kicker + thres_theta, upper_angle])
                # elif angle_bot_to_kicker - thres_theta - lower_angle  < 0.1 and (angle_bot_to_kicker + thres_theta >lower_angle and angle_bot_to_kicker + thres_theta < upper_angle):
                #     angle_bot_to_kicker_list.append([lower_angle, angle_bot_to_kicker - thres_theta])
                #print(bots,state.homePos[bots].x > state.homePos[bot_id].x)
        
        closed_angle_list = range_merge(angle_bot_to_kicker_list)
        open_angles = []
        k = 1
        if len(closed_angle_list):
            open_angles.append([lower_angle, closed_angle_list[0][0], closed_angle_list[0][0]-lower_angle])
            for i in xrange(len(closed_angle_list)-1):
                open_angles.append([closed_angle_list[i][k], closed_angle_list[i+1][1-k], closed_angle_list[i+1][1-k]-closed_angle_list[i][k]])
                k = 1-k
            open_angles.append([closed_angle_list[len(closed_angle_list)-1][1], upper_angle, upper_angle-closed_angle_list[len(closed_angle_list)-1][1]])
            open_angles.sort(key = lambda x:x[2], reverse = True)
        else:
            open_angles.append([lower_angle, upper_angle, upper_angle-lower_angle])

        #print("closed",array(closed_angle_list)*57.2)
        return open_angles[0][2]/(upper_angle-lower_angle)

    









