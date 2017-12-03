import skill_node
import math
import sys

sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../plays_py/scripts/utils')

from config import *
from wrapperpy import *
from obstacle import Obstacle
from geometry import *


POINTPREDICTIONFACTOR = 2

#some constants yet to be defined


def execute(param,state,bot_id, pub):
    obs = Vector_Obstacle()
    for i in range(0,len(state.awayDetected)):
        if state.awayDetected[i] and i != bot_id:
            o = Obstacle()     
            o.x=state.awayPos[i].x
            o.y=state.awayPos[i].y
            o.radius=3.3*BOT_RADIUS
            obs.push_back(o)

    for j in range(0,len(state.awayDetected)):
        if state.homeDetected[j]:
            o = Obstacle()
            o.x=state.homePos[j].x
            o.y=state.homePos[j].y
            o.radius=3.3*BOT_RADIUS
            obs.push_back(o)


    ballfinalpos = Vector2D()
    ballfinalpos.x = int(state.ballPos.x + (state.ballVel.x)/POINTPREDICTIONFACTOR)
    ballfinalpos.y = int(state.ballPos.y + (state.ballVel.y)/POINTPREDICTIONFACTOR)

    point = Vector2D()
    nextWP = Vector2D()
    nextNWP = Vector2D()

    pathplanner = MergeSCurve()

    botPos = Vector2D(int(state.awayPos[bot_id].x),int(state.awayPos[bot_id].y))

    pathplanner.plan(botPos,ballfinalpos,nextWP,nextNWP,obs,len(obs),bot_id,True)


    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
    dist = ballPos.dist(botPos)
    maxDisToTurn = dist - 3.5 * BOT_BALL_THRESH
    angleToTurn = ballPos.normalizeAngle((ballPos.angle(botPos))-(state.homePos[bot_id].theta))

    minReachTime = maxDisToTurn / MAX_BOT_OMEGA
    maxReachTime = maxDisToTurn / MIN_BOT_OMEGA

    minTurnTime = angleToTurn / MAX_BOT_OMEGA
    maxTurnTime = angleToTurn / MIN_BOT_OMEGA

    speed = 0.0
    omega = angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)

    if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
        if omega < 0:
            omega = -MIN_BOT_OMEGA
        else:
            omega = MIN_BOT_OMEGA

    if maxDisToTurn > 0:
        if minTurnTime > maxReachTime:
            speed = MIN_BOT_SPEED
        elif minReachTime > maxTurnTime:
            speed = MAX_BOT_SPEED
        elif minReachTime < minTurnTime:
            speed =  maxDisToTurn / minTurnTime
        elif minTurnTime < minReachTime:
            speed = MAX_BOT_SPEED
    else:
        speed = dist / MAX_FIELD_DIST * MAX_BOT_SPEED

    vec = Vector2D()
    motionAngle = nextWP.angle(botPos)
    theta  = motionAngle - state.awayPos[bot_id].theta

    if param.GoToBallP.intercept == False:
        if dist < DRIBBLER_BALL_THRESH:
            if dist < 2*BOT_BALL_THRESH:
                skill_node.send_command(pub, True, bot_id, 0, 0, 0, 0, True)
            else:
                skill_node.send_command(pub, True, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, True)

        else:
            skill_node.send_command(pub, True, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, False)

    else:
        if dist > BOT_BALL_THRESH:
            skill_node.send_command(pub, True, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), 0, 0, False)
        else:
            skill_node.send_command(pub, True, bot_id, 0, 0, 0, 0,True)


