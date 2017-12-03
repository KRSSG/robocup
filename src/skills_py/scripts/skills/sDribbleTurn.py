import skill_node
from math import *
import sys

sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '/../../../plays_py/scripts/utils')
sys.path.append('/home/vernwalrahul/rahul-krssg/src/skills_py/scripts/skills')

from config import *
from wrapperpy import *
from geometry import *
import sGoToBall
import sTurnToAngle
import skills_union

FRICTION_COEFF = 0.05
ACCN_GRAVITY = 9.80665
ANGLE_THRES =30.0
OMEGA=1

def execute(param, state, bot_id, pub):
    centre=Vector2D(int(param.DribbleTurnP.x), int(param.DribbleTurnP.y))
    ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))
    my_pos=Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
    my_angle=state.homePos[bot_id].theta
    radius=my_pos.dist(centre)                            #approx, use cosine rule to get exact radius, solve quadratic
    phi=atan(FRICTION_COEFF*ACCN_GRAVITY/(OMEGA*OMEGA*radius/1000.0))
    finalslope=my_pos.normalizeAngle(centre.angle(my_pos)-phi)
    dist=my_pos.dist(ballPos)
    turnAngleleft=my_pos.normalizeAngle(finalslope- my_angle)

    print "Final_slope="+str(finalslope*180/pi)+"    phi="+str(phi*180/pi)

    if dist > BOT_BALL_THRESH+10 :
        print "go_to_ball"
        sGoToBall.execute(param, state, bot_id, pub, True)
        return 

    if fabs(turnAngleleft) > SATISFIABLE_THETA/2 : # SATISFIABLE_THETA in config file
        sParam = skills_union.SParam()
        sParam.TurnToAngleP.finalslope=finalslope
        print "turn_to_angle"
        sTurnToAngle.execute(sParam, state, bot_id, pub, True)
        return

    speed=radius*OMEGA
    alpha=my_pos.angle(centre)
    v_x=-speed*sin(my_pos.angle(centre))
    v_y=+speed*cos(my_pos.angle(centre))
    # if(alpha<0):
    #     v_y*=-1
    #     v_x*=-1    
    omega=my_pos.normalizeAngle(OMEGA+phi-2*pi)   
    omega=0 
    theta=state.homePos[bot_id].theta
    v_oy=v_x*cos(theta)+v_y*sin(theta)
    v_ox=v_x*sin(theta)-v_y*cos(theta)  
    print "GOING_____________v_x="+str(v_x)+" v_y="+str(v_y)
    print("Alpha=",alpha*180/pi)
    skill_node.send_command(pub, state.isteamyellow, bot_id, v_ox, v_oy, omega, 0, True)              
    return  