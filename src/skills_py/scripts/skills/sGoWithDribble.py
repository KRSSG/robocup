import skill_node
import math
import sys

sys.path.append('../../../src/navigation_py/scripts/navigation/')
sys.path.append('../../../src/plays_py/scripts/utils/')

from wrapperpy import MergeSCurve, Vector_Obstacle
from obstacle import Obstacle
from config import *
from geometry import Vector2D 
from math import pi, exp

POINTPREDICTIONFACTOR = 2


def debug(param, state, bot_pos, target_pos, nextWP, nextNWP, speed, theta, omega, obs):
    print '#'*50
    print 'Current bot pos: {}, {}'.format(bot_pos.x, bot_pos.y)
    print 'Target  bot pos: {}, {}'.format(param.GoWithDribbleP.x, param.GoWithDribbleP.y)
    print 'NextWP  bot pos: {}, {}'.format(nextWP.x, nextWP.y)
    print 'NextNWP bot pos: {}, {}'.format(nextNWP.x, nextNWP.y)
    print 'speed: {}\ttheta: {}\tomega: {}'.format(speed, theta, omega)
    print 'len(obs): {}'.format(len(obs))
    print 'frame : {}'.format(state.frame_number)
    print '#'*50


def execute(param,state,bot_id, pub):
    point = Vector2D(int(param.GoWithDribbleP.x),int(param.GoWithDribbleP.y))
    myPos = Vector2D(int(state.homePos[bot_id].x),int(state.homePos[bot_id].y))
    distance = point.dist(myPos)
    MAX_BOT_SPEED = 0.7*1800
    speed= 2*distance*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
    from math import *
    theta = myPos.angle(point)
    print(theta)
    theta = point.normalizeAngle(theta + state.homePos[bot_id].theta)
    vel_x = speed*cos(theta)
    vel_y = speed*sin(theta)
    skill_node.send_command(pub, state.isteamyellow, bot_id, vel_x, vel_y, 0, 0,True)
    # obs = Vector_Obstacle()
    # for i in range(0,len(state.homeDetected)):
    #     if state.homeDetected[i] and i != bot_id:
    #         o = Obstacle()     
    #         o.x=state.homePos[i].x
    #         o.y=state.homePos[i].y
    #         o.radius=3.3*BOT_RADIUS
    #         obs.push_back(o)

    # for j in range(0,len(state.awayDetected)):
    #     if state.awayDetected[j]:
    #         o = Obstacle()
    #         o.x=state.awayPos[j].x
    #         o.y=state.awayPos[j].y
    #         o.radius=3.3*BOT_RADIUS
    #         obs.push_back(o)


    # pointPos = Vector2D()
    # pointPos.x = int(param.GoWithDribbleP.x)
    # pointPos.y = int(param.GoWithDribbleP.y)

    # point = Vector2D()
    # nextWP = Vector2D()
    # nextNWP = Vector2D()

    # pathplanner = MergeSCurve()

    # botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
    # ballPos = Vector2D(int(state.ballPos.x),int(state.ballPos.y))
    # thetadiff = botPos.angle(pointPos) - state.homePos[bot_id].theta
    # print(thetadiff,SATISFIABLE_THETA)
    # if thetadiff > SATISFIABLE_THETA or botPos.dist(ballPos)>DRIBBLER_BALL_THRESH and 0:
    #     import sDribbleTurn
    #     print("executing")
    #     param.DribbleTurnP.x = pointPos.x
    #     param.DribbleTurnP.y = pointPos.y
    #     sDribbleTurn.execute(param,state,bot_id,pub)
    # else:
    #     #print "bot no "+str(bot_id)+" in sGoWithDribble currently in "+str(botPos.x)+","+str(botPos.y)
    #     #print "going for "+str(pointPos.x)+","+str(pointPos.y)+" angle="+str(param.GoWithDribbleP.finalslope*180/pi)

    #     pathplanner.plan(botPos,pointPos,nextWP,nextNWP,obs,len(obs),bot_id, True)
    #     v = Vector2D()
    #     distan = botPos.dist(pointPos)
    #     maxDisToTurn = distan 
    #     angleToTurn = v.normalizeAngle((param.GoWithDribbleP.finalslope)-(state.homePos[bot_id].theta))

    #     minReachTime = maxDisToTurn / MAX_BOT_OMEGA
    #     maxReachTime = maxDisToTurn / MIN_BOT_OMEGA

    #     minTurnTime = angleToTurn / MAX_BOT_OMEGA
    #     maxTurnTime = angleToTurn / MIN_BOT_OMEGA

    #     speed = 0.0
    #     omega = angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)

    #     if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
    #         if omega < 0:
    #             omega = -MIN_BOT_OMEGA
    #         else:
    #             omega = MIN_BOT_OMEGA
    # ####################    for testing
    #     # if maxDisToTurn > 0:
    #     #     if minTurnTime > maxReachTime:
    #     #         speed = MIN_BOT_SPEED
    #     #     elif minReachTime > maxTurnTime:
    #     #         speed = MAX_BOT_SPEED
    #     #     elif minReachTime < minTurnTime:
    #     #         speed =  maxDisToTurn / minTurnTime
    #     #     elif minTurnTime < minReachTime:
    #     #         speed = MAX_BOT_SPEED
    #     # else:
    #     #     speed = distan / MAX_FIELD_DIST * MAX_BOT_SPEED    

    # ##############################final

    #     speed= 2*maxDisToTurn*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
    #     if (speed)< 16*MIN_BOT_SPEED:
    #         speed=16*MIN_BOT_SPEED

    #     # omega =0

    #     # distance = pointPos.dist(botPos)
    #     # optimaldistane = distance -590
    #     # if optimaldistane > 0:
    #     #     optimaldistane = 0
    #     # else:
    #     #     optimaldistane*=-1
    #     # speed = 1800*exp(-optimaldistane/90)

    #     vec = Vector2D()
    #     motionAngle = botPos.angle(nextWP)
    #     theta  = motionAngle - state.homePos[bot_id].theta
    #     ballbotdistance = botPos.dist(Vector2D(int(state.ballPos.x),int(state.ballPos.y)))
    #     Dribbler = False
    #     if ballbotdistance<DRIBBLER_BALL_THRESH:
    #         Dribbler =True
    #     #debug(param, state, botPos, pointPos, nextWP, nextNWP, speed, theta, omega, obs)
    #     # print "Motion angle : {}".format(motionAngle)
    #     omeg=0
        # if param.GoWithDribbleP.align == False:
        #     if distan < DRIBBLER_BALL_THRESH:
        #         if distan < 2*BOT_BALL_THRESH:
        #             skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, 0, 0, Dribbler)
        #             turnAngleLeft = v.normalizeAngle((param.GoWithDribbleP.finalslope)-(state.homePos[bot_id].theta)) # Angle left to turn
        
        #             omega = turnAngleLeft * MAX_BOT_OMEGA/(2*math.pi); # Speedup turn 
        #             omega =0
        #             if(omega < MIN_BOT_OMEGA/2 and omega > -MIN_BOT_OMEGA/2): # This is a rare used skill so believe in Accuracy more than speed. Hence reducing minimum Omega
        #                 if(omega < 0): omega = -MIN_BOT_OMEGA/2
        #                 else: omega = MIN_BOT_OMEGA/2
     
        #             #dist = ballPos.dist(botPos)
        #             #if(dist < DRIBBLER_BALL_THRESH):
        #             skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,Dribbler)
        #             #else:
        #             #    skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,False)
        #         else:
        #             skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, True)
        #     else:
        #         skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, Dribbler)
        # else:
        #     if distan > BOT_BALL_THRESH/4:
        #         skill_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), 0, 0, Dribbler)
        #     else:
        #         skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, 0, 0,Dribbler)
        #         turnAngleLeft = v.normalizeAngle((param.GoWithDribbleP.finalslope)-(state.homePos[bot_id].theta)) # Angle left to turn
        
        #         omega = turnAngleLeft * MAX_BOT_OMEGA/(2*math.pi); # Speedup turn 
        #         omega =0 
        #         if(omega < MIN_BOT_OMEGA/2 and omega > -MIN_BOT_OMEGA/2): # This is a rare used skill so believe in Accuracy more than speed. Hence reducing minimum Omega
        #             if(omega < 0): omega = -MIN_BOT_OMEGA/2
        #             else: omega = MIN_BOT_OMEGA/2
     
        #         #dist = ballPos.dist(botPos)
        #             #if(dist < DRIBBLER_BALL_THRESH):
        #         skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,Dribbler)

