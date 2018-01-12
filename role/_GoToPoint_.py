from kubs import kubs, cmd_node
from velocity.run import *
from velocity.run_w import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils.geometry import Vector2D
from utils.config import *
from utils.math_functions import *

import memcache
shared = memcache.Client(BS_ADDRESS,debug=0)

kub = None
start_time = None
GOAL_POINT = None
FLAG_move = False
FLAG_turn = False
rotate = 0

FIRST_CALL = True

prev_state = shared.get('state')
def init(_kub,target,theta):
    global kub,GOAL_POINT,rotate,FLAG_turn,FLAG_move,FIRST_CALL
    kub = _kub
    GOAL_POINT = point_2d()
    rotate = theta
    GOAL_POINT.x = target.x
    GOAL_POINT.y = target.y
    FLAG_move = False
    FLAG_turn = False
    FIRST_CALL = True


def reset():
    global start_time
    start_time = rospy.Time.now()
    start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
    
def execute(startTime,DIST_THRESH,avoid_ball=False):
    global GOAL_POINT, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub,prev_state

    # print DIST_THRESH
    if FIRST_CALL:
        start_time = startTime
        FIRST_CALL = False
        vx_end,vy_end = 0,0


    while not (FLAG_move and FLAG_turn):

        kub.state = shared.get('state')
        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            [vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, kub.kubs_id, GOAL_POINT, kub.state.homePos, kub.state.awayPos, avoid_ball)
            vw = Get_Omega(kub.kubs_id,rotate,kub.state.homePos)
            
            if not vw:
                vw = 0

            if(REPLANNED):
                reset()

            if not vx and not vy:
                vx,vy = vx_end,vy_end
            else:
                vx_end,vy_end = vx,vy

            if abs(normalize_angle(kub.state.homePos[kub.kubs_id].theta-rotate))<ROTATION_FACTOR:
                kub.turn(0)
                FLAG_turn = True
            else:
                kub.turn(vw)

            if dist(kub.state.homePos[kub.kubs_id], GOAL_POINT)<DIST_THRESH :
                kub.move(0,0)
                FLAG_move = True
            else:
                kub.move(vx, vy)

            kub.execute()
            yield kub,GOAL_POINT

        # else:
        #     print "_______False loop__________"


    
    kub.execute()
    kub.execute()

    yield kub,GOAL_POINT



