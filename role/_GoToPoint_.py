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
from krssg_ssl_msgs.srv import *
from utils.functions import *
import math

kub = None
start_time = None
GOAL_POINT = None
FLAG_move = False
FLAG_turn = False
rotate = 0

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
# print("Importing _gotopoint_")
FIRST_CALL = True
vx_end,vy_end = 0,0

#prev_state = shared.get('state')
prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)

print(prev_state)
def init(_kub,target,theta):
    global kub,GOAL_POINT,rotate,FLAG_turn,FLAG_move,FIRST_CALL
    kub = _kub
    GOAL_POINT = Vector2D()
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
    global getState,GOAL_POINT, start_time,FIRST_CALL,FLAG_turn,FLAG_move,kub,prev_state,vx_end,vy_end

    if FIRST_CALL:
        print("First call of _gotopoint_")
        start_time = startTime
        FIRST_CALL = False
        vx_end,vy_end = 0,0


    while not (FLAG_move and FLAG_turn):

        #kub.state = shared.get('state')
        try:
            kub.state = getState(prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)
        print(kub.state)
        if not(prev_state == kub.state):
            prev_state = kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            [vx, vy, vw, REPLANNED] = velocity.run.Get_Vel(start_time, t, kub.kubs_id, GOAL_POINT, kub.state.homePos, kub.state.awayPos, avoid_ball)
            velocity_magnitude = Vector2D(vx,vy).abs(Vector2D(vx,vy))
            if velocity_magnitude > MAX_BOT_SPEED:
                angle_movement = math.atan2(vy,vx)
                # print("_____________Velocity Changed____________")
                vy = MAX_BOT_SPEED*math.sin(angle_movement)
                vx = MAX_BOT_SPEED*math.cos(angle_movement)
                # pass
            # vy = min(vy,MIN_BOT_SPEED)

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
                # print("Angle completed")
                FLAG_turn = True
            else:
                kub.turn(vw)
            # print("Distance ______",dist(kub.state.homePos[kub.kubs_id], GOAL_POINT))
            if dist(kub.state.homePos[kub.kubs_id], GOAL_POINT)<DIST_THRESH :
                kub.move(0,0)
                # print("Distance completed"*200)
                FLAG_move = True
            else:
                # print("Sending velocity",vx,vy)
                kub.move(vx, vy)

            kub.execute()
            yield kub,GOAL_POINT

        # else:
        #     print "_______False loop__________"


    
    kub.execute()
    # kub.execute()

    yield kub,GOAL_POINT



