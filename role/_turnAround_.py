# print("_gotopoint imported")
from kubs import kubs, cmd_node
# print("Importing run")
try:
    velocity.run = reload(velocity.run)
except:
    import velocity.run
# from velocity.run import *
# import velocity.run 
from velocity.run_w import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils.geometry import Vector2D
from utils.config import *
from utils.functions import *
import math

import memcache
shared = memcache.Client(BS_ADDRESS,debug=0)

kub = None
start_time = None
FLAG_turn = False
rotate = 0

# print("Importing _gotopoint_")
FIRST_CALL = True

prev_state = shared.get('state')
def init(_kub,theta):
    global kub,rotate,FLAG_turn,FIRST_CALL
    kub = _kub
    rotate = theta
    FLAG_turn = False
    FIRST_CALL = True


def reset():
    global start_time
    start_time = rospy.Time.now()
    start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
    
def execute(startTime):
    global start_time,FIRST_CALL,FLAG_turn,kub,prev_state

    # print DIST_THRESH
    if FIRST_CALL:
        # print("First call of _gotopoint_")
        start_time = startTime
        FIRST_CALL = False
        vw = 0


    while not FLAG_turn:

        kub.state = shared.get('state')
        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            vw = Get_Omega(kub.kubs_id,rotate,kub.state.homePos)
            
            if not vw:
                vw = 0

            print("rotate : ", rotate)
            print(" not rotate : ", kub.state.homePos[kub.kubs_id].theta)
            print("diff : ",abs(normalize_angle(kub.state.homePos[kub.kubs_id].theta-rotate)))
            if (abs(kub.state.homePos[kub.kubs_id].theta-rotate)<2*ROTATION_FACTOR):
                kub.turn(0)
                print("Angle completed")
                FLAG_turn = True
            else:
                kub.turn(vw)

            kub.execute()
            yield kub,rotate 

        # else:
        #     print "_______False loop__________"


    
    kub.execute()
    # kub.execute()

    yield kub,rotate 



