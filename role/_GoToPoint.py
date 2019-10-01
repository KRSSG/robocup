print "In test GOToPoint"
from kubs import kubs, cmd_node
from velocity.run import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import point_SF
from utils.config import *
from utils.functions import *
from krssg_ssl_msgs.srv import bsServer
import sys

# BOT_ID = int(sys.argv[1])
# print "bot_id received",BOT_ID
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer', bsServer)

GOAL_POINT = point_2d()
GOAL_POINT.x = 1000
GOAL_POINT.y = 1200
REPLANNED = 0
homePos = None
awayPos = None
kub = None
start_time = None
FIRST_CALL = True
FLAG_move = None
target = None


def init(kub_,target_,theta_):
    global start_time,kub,FLAG_move,FIRST_CALL, target
    kub = kub_
    FLAG_move = True
    FIRST_CALL = True
    target = target_
    print("initialised ")
def reset():
    global start_time
    start_time = rospy.Time.now()
    start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)



def execute(start_time_,DIST_THRESH,data,avoid_ball=False):
    global homePos, REPLANNED,FIRST_CALL,FLAG_move
    global awayPos, start_time, BState, kub, target
    print ("safsdf")
    # while FLAG_move:
    state = None
    state = getState(state)
    state = state.stateB 
    if FIRST_CALL:
        start_time = start_time_
        FIRST_CALL = False
        
        if not vx and not vy:
            vx,vy = vx_end,vy_end
        else:
            vx_end,vy_end = vx,vy

        if vx is inf or vy is inf:
            vx,vy = 0,0
            
        if not vw:
            # print "Didn't receive Omega"
            vw = 0

        if(REPLANNED):
            reset()
        
        

        # print vx,vy,vw

        # print radian_2_deg(kub.state.homePos[kub.kubs_id].theta-rotate),radian_2_deg(ROTATION_FACTOR)
        # print dist(kub.state.homePos[kub.kubs_id], GOAL_POINT),DIST_THRESH
        # print kub.state.homePos[kub.kubs_id].x,kub.state.homePos[kub.kubs_id].y
        # print GOAL_POINT.x,GOAL_POINT.y
        # print FLAG_move,"flag mobve    fujfgciuygvb"
        if abs(kub.state.homePos[kub.kubs_id].theta-rotate)<ROTATION_FACTOR:
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
 
        # yield kub
