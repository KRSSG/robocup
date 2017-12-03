from tactic import Tactic
import time
import sys

sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/skills_py/scripts/skills')
sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/plays_py/scripts/utils/')
sys.path.insert(0, '/home/shubham00/krssg-ssl/catkin_ws/src/navigation_py/scripts/navigation')

from geometry import Vector2D 
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
        markBotID = self.param.IntercptP.awayBotID
        markBot = Vector2D(int(state.homePos[markBotID].x), int(state.homePos[markBotID].y))
        botPos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        ballpos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

        passerBotID = -1
        mindis = -1.0
        TEAMSIZE = 5

        for i in range(TEAMSIZE):
            if i != self.bot_id and i != markBotID:
                bot = Vector2D(int(state.homePos[i].x), int(state.homePos[i].y))
                dis = bot.dist(ballpos)
                if 0.0 > mindis:
                    mindis = dis
                    passerBotID = i
                elif dis < mindis:
                    mindis = dis     #ASK
                    passerBotID = i  #ASK

        passerBot = Vector2D(int(state.homePos[passerBotID].x), int(state.homePos[passerBotID].y))

        #goal_point is the point which will be blocked by the intercepter
        OUR_GOAL_X = 500

        goal_point = Vector2D(int(OUR_GOAL_X), int(0))
        destination = Vector2D()
        destinationSlope =Vector2D()

        ballFromBot = ballpos.dist(botPos)
        ballFromPasser = ballpos.dist(passerBot)
        ballFromMark = ballpos.dist(markBot)

        global ballFromOpp

        if ballFromPasser > ballFromMark:
            ballFromOpp = ballFromMark/1.0
        else:
            ballFromOpp = ballFromPasser/1.0

        if ballFromOpp < ballFromBot:
            if self.param.IntercptP.where == 0:
                destination.x =  int(0.7*markBot.x +0.3*passerBot.x)
                destination.y =  int(0.7*markBot.y+0.3*passerBot.y)                           #0.7 * markBot + 0.3 * passerBot
            else:
                destination.x =  int(0.7*markBot.x +0.3*goal_point.x)
                destination.y =  int(0.7*markBot.y+0.3*goal_point.y)

            destinationSlope = passerBot.angle(botPos)
            pointFromBot = botPos.dist(destination)

            global iState

            if pointFromBot > BOT_BALL_THRESH:
                iState = 1    #"GOTOPOSITION"
            else:
                if pointFromBot <= DRIBBLER_BALL_THRESH:
                    iState = 2   #"GOTOBALL"
                else:
                    iState = 1  #"GOTOPOSTION"
        else:
            if ballFromBot > DRIBBLER_BALL_THRESH:
                iState =   2  #"GOTOBALL"
            else:
                iState =  3   #"CLEAR"

        if iState is 1:
                #call here gotoPosition
            self.sParam.GoToPointP.x = destination.x;
            self.sParam.GoToPointP.y = destination.y;
            self.sParam.GoToPointP.finalVelocity = MAX_BOT_SPEED;
            self.sParam.GoToPointP.finalslope = destinationSlope;
            self.sParam.GoToPointP.align = False;

            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return
        elif iState is 2:
                #call here gotoBall
            self.sParam.GoToBallP.intercept = False
            sGoToBall.execute(self.sParam, state, self.bot_id, pub)
            return

        elif iState is 3:
                #call here clear
            self.sParam.KickP.power = 10
            sKick.execute(self.sParam, state, self.bot_id, pub)
            return 
    def isComplete(self, state):
        botpos = Vector2D(int(state.homePos[self.bot_id].x),int(state.homePos[self.bot_id].y))
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























