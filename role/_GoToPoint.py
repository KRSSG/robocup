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
    global GOAL_POINT, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub

    # print DIST_THRESH
    if FIRST_CALL:
        start_time = startTime
        FIRST_CALL = False

    while not (FLAG_move and FLAG_turn):
        # print not (FLAG_move and FLAG_turn)

        kub.state = shared.get('state')
        # print " in _ ",kub.state.ballPos.x
        
        t = rospy.Time.now()
        t = t.secs + 1.0*t.nsecs/pow(10,9)

        [vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, kub.kubs_id, GOAL_POINT, kub.state.homePos, kub.state.awayPos, avoid_ball)
        vw = Get_Omega(kub.kubs_id,rotate,kub.state.homePos)
        
        if not vw:
            # print "Didn't receive Omega"
            vw = 0

        if(REPLANNED):
            reset()
        kub.move(vx, vy)
        kub.turn(vw)

        # print vx,vy,vw

        # print radian_2_deg(kub.state.homePos[kub.kubs_id].theta-rotate),radian_2_deg(ROTATION_FACTOR)
        # print dist(kub.state.homePos[kub.kubs_id], GOAL_POINT),DIST_THRESH
        # print kub.state.homePos[kub.kubs_id].x,kub.state.homePos[kub.kubs_id].y
        # print GOAL_POINT.x,GOAL_POINT.y
        # print FLAG_move,"flag mobve    fujfgciuygvb"
        if abs(kub.state.homePos[kub.kubs_id].theta-rotate)<ROTATION_FACTOR:
            kub.turn(0)
            FLAG_turn = True


        if dist(kub.state.homePos[kub.kubs_id], GOAL_POINT)<DIST_THRESH :
            kub.move(0,0)
            FLAG_move = True
        
        kub.execute()
        yield kub,GOAL_POINT


    
    kub.execute()
    kub.execute()

    # print "sd", not (FLAG_move and FLAG_turn)
    # print kub.get_pos().x,kub.get_pos().y,GOAL_POINT.x,GOAL_POINT.y,
    yield kub,GOAL_POINT


