import skill_node
import math
import sys

sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../plays_py/scripts/utils')

from config import *
from wrapperpy import *
from obstacle import Obstacle
from geometry import *

def execute(param, state, bot_id, pub):
    DEFEND_RADIUS = 50.0
    ob=Vector2D()
    botPos=Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
    point_ball_angle = math.atan2(state.ballPos.y-param.DefendPointP.y,state.ballPos.x-param.DefendPointP.x)
    dpoint =Vector2D(int(param.DefendPointP.x + (param.DefendPointP.radius)*math.cos(point_ball_angle)) , int(param.DefendPointP.y + (param.DefendPointP.radius)*math.sin(point_ball_angle)))
    obs = Vector_Obstacle()
    for i in range(0,len(state.homeDetected)):
        if state.homeDetected[i] and i != bot_id:
            o = Obstacle()     
            o.x=state.homePos[i].x
            o.y=state.homePos[i].y
            o.radius=3.3*BOT_RADIUS
            obs.push_back(o)

    for j in range(0,len(state.awayDetected)):
        if state.awayDetected[j]:
            o = Obstacle()
            o.x=state.awayPos[j].x
            o.y=state.awayPos[j].y
            o.radius=3.3*BOT_RADIUS
            obs.push_back(o)

    point=Vector2D()
    nextWP = Vector2D()
    nextNWP =Vector2D()

    # calling of pathplanner has to be noticed later

    pathPlanner=MergeSCurve() 
    pathPlanner.plan(botPos,dpoint,nextWP,nextNWP,obs,len(state.homeDetected) + len(state.awayDetected),bot_id,True)
    motionAngle = nextWP.angle(botPos)

    if nextNWP.valid() is True :            # this has to be checked 
        finalSlope = nextNWP.angle(nextWP)
    else :
        finalSlope = nextWP.angle(botPos)

    finalSlope=point_ball_angle

    turnAngleLeft = ob.normalizeAngle(finalSlope - state.homePos[bot_id].theta)
    omega = turnAngleLeft * MAX_BOT_OMEGA / (2 * math.pi)    #Speedup turn
    if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA :
        if omega < 0 :
            omega = -MIN_BOT_OMEGA
        else : 
            omega = MIN_BOT_OMEGA
    dist = nextWP.dist(botPos)               # Distance of next waypoint from the bot
    theta =  motionAngle - state.homePos[bot_id].theta   #Angle of dest with respect to bot's frame of reference
    profileFactor = (dist * 2 / MAX_FIELD_DIST) * MAX_BOT_SPEED

    if profileFactor < MIN_BOT_SPEED :
        profileFactor = MIN_BOT_SPEED
    elif profileFactor > MAX_BOT_SPEED :
		profileFactor = MAX_BOT_SPEED
    print "HEre!!"
    if dist < BOT_POINT_THRESH :
        if (turnAngleLeft) > -DRIBBLER_BALL_ANGLE_RANGE  and (turnAngleLeft) < DRIBBLER_BALL_ANGLE_RANGE :
            skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, 0, 0, True) # previous comment here was kick , needs be checked 
        else :
            skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0, True)
    else :
        skill_node.send_command(pub, state.isteamyellow , bot_id,profileFactor * math.sin(-theta) , profileFactor * math.cos(-theta), omega, 0, False)


