from utils.wrapperpy import MergeSCurve, Vector_Obstacle
from utils.obstacle import Obstacle
from utils.config import *
from utils.geometry import Vector2D
from utils.functions import *
import math

import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from kubs import kubs

pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=False)

POINTPREDICTIONFACTOR = 2


def GoToPoint(point,kub):

    state = shared.get('state')
    obs = Vector_Obstacle()
    for i in range(0,len(state.homeDetected)):
        if state.homeDetected[i] and i != kub.kubs_id:
            o = Obstacle()
            o.x=state.homePos[i].x
            o.y=state.homePos[i].y
            o.radius=2.5*BOT_RADIUS
            obs.push_back(o)

    for j in range(0,len(state.awayDetected)):
        if state.awayDetected[j]:
            o = Obstacle()
            o.x=state.awayPos[j].x
            o.y=state.awayPos[j].y
            o.radius=2.5*BOT_RADIUS
            obs.push_back(o)


    pointPos = Vector2D()
    pointPos.x = int(point.x)
    pointPos.y = int(point.y)
    point = Vector2D()
    nextWP = Vector2D()
    nextNWP = Vector2D()

    pathplanner = MergeSCurve()

    botPos = Vector2D(int(state.homePos[kub.kubs_id].x), int(state.homePos[kub.kubs_id].y))

    pathplanner.plan(botPos,pointPos,nextWP,nextNWP,obs,len(obs),kub.kubs_id,not state.isteamyellow)

   
    dist = pointPos.dist(botPos)
    maxDisToTurn = dist -  DRIBBLER_BALL_THRESH
    angleToTurn = normalize_angle((pointPos.angle(botPos))-(state.homePos[kub.kubs_id].theta))
    print dist

    minReachTime = maxDisToTurn / MAX_BOT_SPEED
    maxReachTime = maxDisToTurn / MIN_BOT_SPEED

    minTurnTime = angleToTurn / MAX_BOT_OMEGA
    maxTurnTime = angleToTurn / MIN_BOT_OMEGA

    speed = 0.0
    omega = angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)

    if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
        if omega < 0:
            omega = -MIN_BOT_OMEGA
        else:
            omega = MIN_BOT_OMEGA

   
    speed= 2*maxDisToTurn*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
    if (speed)< 2*MIN_BOT_SPEED:
        speed=2*MIN_BOT_SPEED
    if  (speed > MAX_BOT_SPEED):
        speed=MAX_BOT_SPEED    

    vec = Vector2D()
    motionAngle = nextWP.angle(botPos)
    theta  = motionAngle - state.homePos[kub.kubs_id].theta
    theta  = normalize_angle(theta)

    # omega = 0.0



    if dist < DRIBBLER_BALL_THRESH:
        if dist < 1*BOT_BALL_THRESH:
            kub.move(0,0)
            kub.turn(omega)
            kub.kick(7)
            kub.dribble(True)
            # print "in 1"
            # skill_node.send_command(pub, state.isteamyellow, kub.kubs_id, 0, 0, omega, 0, True)
        else:
            kub.move(speed * math.sin(-theta), speed * math.cos(-theta))
            kub.turn(omega)
            kub.dribble(True)
            # print "in 2"
            # skill_node.send_command(pub, state.isteamyellow, kub.kubs_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, True)

    else:
        kub.move(speed * math.sin(-theta), speed * math.cos(-theta))
        kub.turn(omega)
        # print "in 3"
    print kub.vx,kub.vy,kub.vw
    kub.execute()
        # skill_node.send_command(pub, state.isteamyellow, kub.kubs_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, False)


# def main(kub):
#     # state = shared.get('state')
#     # kub = kubs.kubs(id_,state,pub)
#     GoToPoint(Vector2D(0,0),kub)

 
# # def main():
# #     rospy.init_node('node_new',anonymous=False)
# #     start_time = rospy.Time.now()
# #     start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

# #     # rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)

# #     while True:
# #         state=shared.get('state')   
# #         if state:
# #             function(0,state)
# #             # break

# if __name__ == '__main__':
#     main()