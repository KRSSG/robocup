from tactic import Tactic
import time
import sys
from math import *
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
from config import *
from numpy import array, inf

from geometry import * 
from math import *
import skills_union
import obstacle
import sGoToPoint
from copy import copy


OPTIMAL_DISTANCE = HALF_FIELD_MAXX*3/ 5
BOT_TO_BOT_OPTIMAL_DIST  = HALF_FIELD_MAXY


from isPossible import isPossible

class TMidFielderX(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TMidFielderX, self).__init__( bot_id, state, param)
        self.sParams = skills_union.SParam()


    def OpenAngle(self, state ,bot_id):
        
        kickerBot = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))

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

        Upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(HALF_FIELD_MAXY))
        Lower_limit = Vector2D(int(-HALF_FIELD_MAXX), int(HALF_FIELD_MAXY*-1.0))

        self.upper_angle = atan2((Upper_limit.y-state.homePos[bot_id].y)*1.0, (Upper_limit.x-state.homePos[bot_id].x))
        self.lower_angle = atan2((Lower_limit.y-state.homePos[bot_id].y)*1.0, (Lower_limit.x-state.homePos[bot_id].x))
        #print("chuu ",57.2*self.lower_angle,57.2*self.upper_angle)
        away_in_our_side = [x for x in xrange(6)]
        angle_bot_to_kicker_list = []

        for bots in away_in_our_side:
            angle_bot_to_kicker = atan2((state.awayPos[bots].y-state.homePos[bot_id].y)*1.0, (state.awayPos[bots].x-state.homePos[bot_id].x))
            #print(bots, angle_bot_to_kicker,"gf")
            dist_of_bot = kickerBot.dist(Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y)))
            thres_theta = asin(BOT_RADIUS*1.0/ dist_of_bot)
            #if state.awayPos[bots].x > state.homePos[bot_id].x :
            #print(angle_bot_to_kicker*57.2)
            if angle_bot_to_kicker - thres_theta > self.lower_angle and angle_bot_to_kicker + thres_theta < self.upper_angle:
                angle_bot_to_kicker_list.append([angle_bot_to_kicker - thres_theta, angle_bot_to_kicker + thres_theta])

        closed_angle_list = range_merge(angle_bot_to_kicker_list)
        #print(array(closed_angle_list)*57.2)
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

        return open_angles



    


    def execute(self, state, pub, attackerX_id = -1, bots = [-1,-1,-1], stop = False):
        # print("dsufgfghgfdgjdgfjfvdsgfhdsf_________")
        #attacker_id = self.bot_id

        if stop:
            import sStop
            #sStop.execute(self.sParams, state, self.bot_id, pub)
            #return
            # for i in range(3):
            #     if self.bot_id is bots[i]:


        open_angles = self.OpenAngle(state, attackerX_id)

        #print(state.homePos[attackerX_id].x, state.homePos[attackerX_id].y,attackerX_id)
        Angles =[]
        #print(array(open_angles)*57.2)
        if len(open_angles) is 2 :
            Angles.append([open_angles[0][0], (open_angles[0][0]+open_angles[0][1])*0.5, (open_angles[0][0]+open_angles[0][1])*0.5-open_angles[0][0]])
            Angles.append([(open_angles[0][0]+open_angles[0][1])*0.5, open_angles[0][1], open_angles[0][1]-(open_angles[0][0]+open_angles[0][1])*0.5])
            Angles.append(open_angles[1])
        if len(open_angles) is 1:
            Angles.append([open_angles[0][0], open_angles[0][0]*0.66+open_angles[0][1]*0.33, open_angles[0][0]*0.66+open_angles[0][1]*0.33-open_angles[0][0]])
            Angles.append([open_angles[0][0]*0.66+open_angles[0][1]*0.33, open_angles[0][0]*0.33+open_angles[0][1]*0.66, open_angles[0][0]*0.33+open_angles[0][1]*0.66-open_angles[0][0]*0.66+open_angles[0][1]*0.33])
            Angles.append([open_angles[0][0]*0.33+open_angles[0][1]*0.66, open_angles[0][1], open_angles[0][1]-open_angles[0][0]*0.33+open_angles[0][1]*0.66])
        if len(open_angles)>2 :
            Angles = open_angles[:3] 
        #print("final angles")
        #print(array(Angles)*57.2)
        
        for i in range(3):
            if self.bot_id is bots[i]:
                separation_factor = i-1
                angle_bot_to_kicker = Angles[i]

        
        theta = (angle_bot_to_kicker[1]+angle_bot_to_kicker[0])/2
        
        #print(self.bot_id, theta*57.2,tan(theta), array(angle_bot_to_kicker)*57.2)


        x0 = state.homePos[attackerX_id].x
        y0 = state.homePos[attackerX_id].y
        r  = OPTIMAL_DISTANCE


        if theta < -pi/2:
            c = -1
        elif theta > -pi/2 and theta < 0:
            c = 1
        elif theta > 0 and theta < pi/2 :
            c = 1
        elif theta > pi/2 :
            c = -1
        #dx = dy = 1.0
        #print(self.bot_id, x0, y0 , theta*57.2, attackerX_id)
        X = x0 + c*r*cos(theta)
        Y = y0 + c*r*sin(theta)
        score = 0

        X_f = X 
        Y_f = Y

        #if not (X <= HALF_FIELD_MAXX and X >= -HALF_FIELD_MAXX*0.5 and fabs(Y) <= HALF_FIELD_MAXY):
        

        


        #print(self.bot_id, X, Y , theta*57.2)
        attX_pos = Vector2D(int(state.homePos[attackerX_id].x), int(state.homePos[attackerX_id].y))
        possiblityX = isPossible(self.bot_id, state)
        while X <= HALF_FIELD_MAXX*0.5 and X >= -HALF_FIELD_MAXX*0.5 and fabs(Y) <=HALF_FIELD_MAXY:
            temp_score = possiblityX.GoalScore(state, X, Y, attackerX_id)
            temp_point = Vector2D(int(X), int(Y))
            if attX_pos.dist(temp_point) <= BOT_TO_BOT_OPTIMAL_DIST :
                print("_____________in break____________")
                break
            if temp_score > score:
                score = temp_score 
                X_f = X
                Y_f = Y
            #print(self.bot_id,X,Y,score)
            r += BOT_RADIUS
            X = c*r*cos(theta)
            Y = c*r*sin(theta)
        #print(self.bot_id, X_f, Y_f , theta*57.2)
        if (X_f >= HALF_FIELD_MAXX - DBOX_HEIGHT or X_f <= -HALF_FIELD_MAXX*0.5) and fabs(Y_f) >= HALF_FIELD_MAXY:
            X_f = DBOX_HEIGHT*separation_factor
            Y_f = HALF_FIELD_MAXY*0.8*separation_factor

        elif X_f >= HALF_FIELD_MAXX - DBOX_HEIGHT or X_f <= -HALF_FIELD_MAXX*0.5:
            X_f = DBOX_HEIGHT*separation_factor
        elif fabs(Y) >= HALF_FIELD_MAXY :
            Y_f = HALF_FIELD_MAXY*0.6*separation_factor

        useless = Vector2D(0,0)
        print(self.bot_id, X_f, Y_f , theta*57.2)
        self.sParams.GoToPointP.x  =  X_f
        self.sParams.GoToPointP.y  =  Y_f
        self.sParams.GoToPointP.finalslope  =  useless.normalizeAngle(state.homePos[attackerX_id].theta + pi)
        sGoToPoint.execute(self.sParams, state, self.bot_id, pub)



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

                    