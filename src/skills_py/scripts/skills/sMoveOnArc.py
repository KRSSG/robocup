import skill_node
import math
import sys


sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../plays_py/scripts/utils')

from config import *
from wrapperpy import *
from geometry import *
import sGoToPoint

FRICTION_COEFF = 0.05
ACCN_GRAVITY = 9.80665
ANGLE_THRES =4*math.pi/180
RADIUS_THRESH=BOT_RADIUS

def execute(param, state, bot_id, pub):
    centre=Vector2D(int(param.MoveOnArcP.centrex), int(param.MoveOnArcP.centrey))
    finalPoint=Vector2D(int(param.MoveOnArcP.finalx), int(param.MoveOnArcP.finaly))
    botPos=Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
    ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))

    print "\n\n\n\n\n\n"
    print " in MoveOnArc bot no "+str(bot_id)+" got final point "+str(finalPoint.x)+str(finalPoint.y)

    if(math.fabs(botPos.dist(centre)-finalPoint.dist(centre))>RADIUS_THRESH):
        param.GoToPointP.x=finalPoint.x
        param.GoToPointP.y=finalPoint.y
        param.GoToPointP.finalslope=ballPos.angle(finalPoint)

        sGoToPoint.execute(param, state, bot_id, pub)
        return

    print "HEEEEEEEEEEEEEEEEEEEEE"    
    Radius=botPos.dist(centre)
    AngleLeft=botPos.normalizeAngle(finalPoint.angle(centre)-botPos.angle(centre))

    print " botPos="+str(botPos.x)+","+str(botPos.y)
    print " finalPoint="+str(finalPoint.x)+","+str(finalPoint.y)
    print " AngleLeft="+str(AngleLeft*180/math.pi)
    print " Radius="+str(Radius)
    print " centre="+str(centre.x)+","+str(centre.y)
    
    
    speed=MAX_BOT_SPEED*AngleLeft/(2*math.pi)

    if(speed<16*MIN_BOT_SPEED):
        speed=16*MIN_BOT_SPEED

    v_x=-speed*math.sin(botPos.angle(centre))
    v_y=+speed*math.cos(botPos.angle(centre))
    if(AngleLeft<0):
        v_y*=-1
        v_x*=-1

    theta=state.homePos[bot_id].theta
    v_oy=v_x*math.cos(theta)+v_y*math.sin(theta)
    v_ox=v_x*math.sin(theta)-v_y*math.cos(theta)


    print " v_x="+str(v_x)+" v_y="+str(v_y)
    
    turnAngleLeft = botPos.normalizeAngle(ballPos.angle(finalPoint)-(state.homePos[bot_id].theta)) # Angle left to turn
    
    omega = turnAngleLeft * MAX_BOT_OMEGA/(2*math.pi); # Speedup turn 
    if(omega < MIN_BOT_OMEGA/2 and omega > -MIN_BOT_OMEGA/2): # This is a rare used skill so believe in Accuracy more than speed. Hence reducing minimum Omega
        if(omega < 0): omega = -MIN_BOT_OMEGA/2
        else: omega = MIN_BOT_OMEGA/2
 
                #dist = ballPos.dist(botPos)
                #if(dist < DRIBBLER_BALL_THRESH):
                #skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,False)


    #omega=0
    if math.fabs(AngleLeft)<ANGLE_THRES:
        print "0 vel "
        skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0, True)
    else:
        print "v_ox="+str(v_ox)+" v_oy="+str(v_oy)
        skill_node.send_command(pub, state.isteamyellow, bot_id, v_ox, v_oy, omega, 0, True)              
