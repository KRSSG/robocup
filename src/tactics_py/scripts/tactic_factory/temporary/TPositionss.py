from tactic import Tactic
import time
import sys

sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/skills_py/scripts/skills')
sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/plays_py/scripts/utils/')
sys.path.insert(0, '/home/shubham00/krssg-ssl/catkin_ws/src/navigation_py/scripts/navigation/src')
sys.path.insert(0, '/home/shubham00/krssg-ssl/catkin_ws/src/navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
'''
import sGoToPoint
import sGoToBall
import sDribbleTurn
'''
import obstacle
import sGoToPoint
import sDribbleTurn
import sDefendPoint
import sGoToBall
#import sTurnToPoint
import sKickToPoint
import sKick




class TPosition(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()



    def execute(self, state, pub):
        #add all parameters 
        markBotID = 3#self.param.IntercptP.awayBotID
        markBot = Vector2D(state.homePos[markBotID].x, state.homePos[markBotID].y)
        botPos = Vector2D(state.homePos[self.bot_id].x, state.homePos[self.bot_id].y)
        ballpos = Vector2D(state.ballPos.x, state.ballPos.y)

        passerBotID = -1
        mindis = -1.0
        TEAMSIZE = 5

        for i in range(TEAMSIZE):
            if i != self.bot_id and i != markBotID:
                bot = Vector2D(state.homePos[i].x, state.homePos[i].y)
                dis = botPos.dist(ballpos)
                if 0.0 > mindis:
                    mindis = dis
                    passerBotID = i
                elif dis < mindis:
                    mindis = dis     #ASK
                    passerBotID = i  #ASK

        passerBot = Vector2D(state.homePos[passerBotID].x, state.homePos[passerBotID].y)

        #goal_point is the point which will be blocked by the intercepter
        OUR_GOAL_X = 0.0
        goal_point = Vector2D(OUR_GOAL_X, 0.0)
        destination = Vector2D()
        destinationSlope =Vector2D()

        ballFromBot = ballpos.dist(botPos)
        ballFromPasser = ballpos.dist(passerBot)
        ballFromMark = ballpos.dist(markBot)

        print("at line 71")

        global ballFromOpp

        if ballFromPasser > ballFromMark:
            ballFromOpp = ballFromMark/1.0
        else:
            ballFromOpp = ballFromPasser/1.0

        if ballFromOpp < ballFromBot:
            if self.param.IntercptP.where == 0:
                destination = markBot.__mul__(0.7) + passerBot.__mul__(0.3)
            else:
                destination = markBot.__mul__(0.7) + goal_point.__mul__(0.3)

            destinationSlope = passerBot.angle(botPos)
            pointFromBot = botPos.dist(destination)

            global iState

            if pointFromBot > BOT_BALL_THRESH:
                iState = "GOTOPOSITION"
            else:
                if pointFromBot <= DRIBBLER_BALL_THRESH:
                    iState = "GOTOBALL"
                else:
                    iState = "GOTOPOSTION"
        else:
            if ballFromBot > DRIBBLER_BALL_THRESH:
                iState = "GOTOBALL"
            else:
                iState = "CLEAR"

        print("at line 104")
        print(iState)

        if iState == "GOTOPOSITION":
                #call here gotoPosition
            self.sParam.GoToPointP.x = destination.x
            self.sParam.GoToPointP.y = destination.y
            self.sParam.GoToPointP.finalVelocity = MAX_BOT_SPEED
            self.sParam.GoToPointP.finalslope = destinationSlope
            self.sParam.GoToPointP.align = False
            print("under pos")

            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            print("After calling sGOToPoint")
            return
        elif iState == "GOTOBALL":
                #call here gotoBall
            print("under pos")
            self.sParam.GoToBallP.intercept = False
            sGoToBall.execute(self.sParam, state, self.bot_id, pub)
            print("After calling sGOToBall")
            return

        elif iState == "CLEAR":
                #call here clear
            self.sParam.KickP.power = 10
            print("under pos")
            sKick.execute(self.sParam, state, self.bot_id, pub)
            print("After calling kick")
            return 


    def isComplete(self, state):
        botpos = Vector2D(state.homePos[self.bot_id].x,state.homePos[self.bot_id].y)
        ballpos = Vector2D(state.ballPos.x, state.ballPos.y)
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























