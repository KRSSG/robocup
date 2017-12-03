import skill_node
import math
import sys

sys.path.append('../../../navigation_py/scripts/navigation/')
sys.path.append('../../../plays_py/scripts/utils/')

from wrapperpy import MergeSCurve, Vector_Obstacle
from obstacle import Obstacle
from config import *
from geometry import Vector2D 
from math import pi

POINTPREDICTIONFACTOR = 2


def debug(param, state, bot_pos, target_pos, nextWP, nextNWP, speed, theta, omega, obs):
    print '#'*50
    print 'Current bot pos: {}, {}'.format(bot_pos.x, bot_pos.y)
    print 'Target  bot pos: {}, {}'.format(param.GoToPointP.x, param.GoToPointP.y)
    print 'NextWP  bot pos: {}, {}'.format(nextWP.x, nextWP.y)
    print 'NextNWP bot pos: {}, {}'.format(nextNWP.x, nextNWP.y)
    print 'speed: {}\ttheta: {}\tomega: {}'.format(speed, theta, omega)
    print 'len(obs): {}'.format(len(obs))
    print 'frame : {}'.format(state.frame_number)
    print '#'*50


def execute(param,state,bot_id, pub,dribller = False, ball_obstacle=False):
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
    if(ball_obstacle):
            print "HOLA_____BALL,OBSTACLE"
            o=Obstacle()
            o.x=state.ballPos.x
            o.y=state.ballPos.y
            o.radius=2*BOT_RADIUS
            obs.push_back(o)          

    pointPos = Vector2D()
    pointPos.x = int(param.GoToPointP.x)
    pointPos.y = int(param.GoToPointP.y)
    point = Vector2D()
    nextWP = Vector2D()
    nextNWP = Vector2D()

    pathplanner = MergeSCurve()

    botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
   
    pathplanner.plan(botPos,pointPos,nextWP,nextNWP,obs,len(obs),bot_id, True)
    v = Vector2D()
    distan = botPos.dist(pointPos)
    maxDisToTurn = distan 
    angleToTurn = v.normalizeAngle((param.GoToPointP.finalslope)-(state.homePos[bot_id].theta))

    minReachTime = maxDisToTurn / MAX_BOT_OMEGA
    maxReachTime = maxDisToTurn / MIN_BOT_OMEGA

    minTurnTime = angleToTurn / MAX_BOT_OMEGA
    maxTurnTime = angleToTurn / MIN_BOT_OMEGA

    speed = 0.0
    omega = 2*angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)                 

    if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
        if omega < 0:
            omega = -1.5*MIN_BOT_OMEGA
        else:
            omega = +1.5*MIN_BOT_OMEGA

    from math import exp

    speed= 2*maxDisToTurn*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
    if (speed)< 2*MIN_BOT_SPEED:
        speed=2*MIN_BOT_SPEED
    if  (speed > MAX_BOT_SPEED):
        speed=MAX_BOT_SPEED    
    # omega = 0
  
    vec = Vector2D()


    motionAngle = nextWP.angle(botPos)
    theta  = motionAngle - state.homePos[bot_id].theta
    if param.GoToPointP.align == False:
        if distan < DRIBBLER_BALL_THRESH:
            if distan < BOT_BALL_THRESH/4.0:
                skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,dribller)
              
            else:
                skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
        else:
            skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
    else:
        if distan > BOT_BALL_THRESH/4.0:
            skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
        else:
            skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,dribller)

