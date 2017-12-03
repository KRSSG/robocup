from tactic import Tactic
import time
import sys

sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/skills_py/scripts/skills')
sys.path.append('/home/shubham00/krssg-ssl/catkin_ws/src/plays_py/scripts/utils/')
sys.path.insert(0, '/home/shubham00/krssg-ssl/catkin_ws/src/navigation_py/scripts/navigation')

from geometry import Vector2D 
import skills_union
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
        markBotID = self.param.InterceptP.awayBotID
        markBot = Vector2D(int(state.homePos[markBotID].x), int(state.homePos[markBotID].y))
        botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
        ballpos = Vector2D(int(state.ballpos.x), int(state.ballpos.y))

        passerBotID = -1
        mindis = -1.0
        TEAMSIZE = 5

        for i in range(TEAMSIZE):
            if i != bot_id and i != markBotID:
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

        ballFromBot = ballpos.dis(botPos)
        ballFromPasser = ballpos.dist(passerBot)
        ballFromMark = ballpos.dist(markBot)

        global ballFromOpp

        if ballFromPasser > ballFromMark:
            ballFromOpp = ballFromMark/1.0
        else:
            ballFromOpp = ballFromPasser/1.0

        if ballFromOpp < ballFromBot:
            if tParam.InterceptP.where == 0:
                destination.x =  int(0.7*markBot.x +0.3*passerBot.x)
                destination.y =  int(0.7*markBot.y+0.3*passerBot.y)                           #0.7 * markBot + 0.3 * passerBot
            else:
                destination.x =  int(0.7*markBot.x +0.3*goal_point.x)
                destination.y =  int(0.7*markBot.y+0.3*goal_point.y)

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

            if iState = "GOTOPOSTION":
                #call here gotoPosition
                sParam.GoToPointP.x = destination.x;
                sParam.GoToPointP.y = destination.y;
                sParam.GoToPointP.finalVelocity = MAX_BOT_SPEED;
                sParam.GoToPointP.finalslope = destinationSlope;
                sParam.GoToPointP.align = false;

                sGoToPoint.execute(sParam, state, bot_id, pub)
                return
            elif iState = "GOTOBALL":
                #call here gotoBall
                sParam.GoToBallP.intercept = False
                sGoToBall.execute(sParam, state, bot_id, pub)
                return

            elif iState = "CLEAR":
                #call here clear
                sParam.KickP.power = 10
                sKick.execute(sParam, state, bot_id, pub)
                return 























