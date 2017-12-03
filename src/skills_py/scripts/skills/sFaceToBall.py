import skill_node
import math
import sys

sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../plays_py/scripts/utils')

from config import *
from wrapperpy import *
from geometry import *
from math import pi, fabs

def execute(param, state, bot_id, pub):
    
    botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

    vec = Vector2D()
    finalslope = ballPos.angle(botPos)
    turnAngleLeft = vec.normalizeAngle(finalslope - state.homePos[bot_id].theta)  #Angle left to turn
    omega =  1*(math.pi)*turnAngleLeft *MAX_BOT_OMEGA / (2 * math.pi)  #Speedup turn

    print "bot_id="+str(bot_id)
    print "my_theta="+str(state.homePos[bot_id].theta)
    print "Turn Angle Left="+str(turnAngleLeft)

    if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
        print(omega, MIN_BOT_OMEGA,"omegaaaaaaaaa")
        if omega < 0:
            omega = -MIN_BOT_OMEGA
        else:
            omega = MIN_BOT_OMEGA


    v_x = omega * BOT_BALL_THRESH * 1.5
    dist = ballPos.dist(botPos)

    if(fabs(turnAngleLeft)<SATISFIABLE_THETA/2.0):
        omega=0
    
    if dist < DRIBBLER_BALL_THRESH * 2:
        skill_node.send_command(pub, state.isteamyellow, bot_id, v_x, 0, omega, 0, True)
    else:
        skill_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0, False)
