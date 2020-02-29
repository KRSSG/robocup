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
from kubs import kubs,cmd_node

pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

# from krssg_ssl_msgs.srv import bsServer
# rospy.wait_for_service('bsServer',)
# getState = rospy.ServiceProxy('bsServer', bsServer)

POINTPREDICTIONFACTOR = 2

kub = kubs.kubs(0, pub)


def GoToPoint(state,point,kub,orient_theta=None):
  # state = None
  #   try:
  #     state = getState(state)
  #   except rospy.ServiceException, e:
  #     print e
  #   if state:
  #     state = state.stateB
  bot_id = 0
  dribller = 0
  obs = Vector_Obstacle()
  for i in range(0,len(state.homeDetected)):
      if state.homeDetected[i] and i != bot_id:
          o = Obstacle()     
          o.x=state.homePos[i].x
          o.y=state.homePos[i].y
          o.radius=3.3*BOT_RADIUS
          obs.push_back(o)

  for j in range(0,len(state.awayDetected)):
      if state.awayDetected[j]:
          o = Obstacle()
          o.x=state.awayPos[j].x
          o.y=state.awayPos[j].y
          o.radius=3.3*BOT_RADIUS
          obs.push_back(o)


  pointPos = Vector2D()
  pointPos.x = int(point.x)
  pointPos.y = int(point.y)
  point = Vector2D()
  nextWP = Vector2D()
  nextNWP = Vector2D()

  pathplanner = MergeSCurve()

  botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
 
  pathplanner.plan(botPos,pointPos,nextWP,nextNWP,obs,len(obs),bot_id, True)
  v = Vector2D()
  distan = botPos.dist(pointPos)
  maxDisToTurn = distan 
  angleToTurn = v.normalizeAngle((orient_theta)-(state.homePos[bot_id].theta))

  minReachTime = maxDisToTurn / MAX_BOT_OMEGA
  maxReachTime = maxDisToTurn / MIN_BOT_OMEGA

  minTurnTime = angleToTurn / MAX_BOT_OMEGA
  maxTurnTime = angleToTurn / MIN_BOT_OMEGA

  speed = 0.0
  omega = 2*angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)                 

  if omega < MIN_BOT_OMEGA and omega > -MIN_BOT_OMEGA:
      if omega < 0:
          omega = -MIN_BOT_OMEGA
      else:
          omega = MIN_BOT_OMEGA

  from math import exp

  speed= 2*maxDisToTurn*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
  if (speed)< 2*MIN_BOT_SPEED:
      speed=2*MIN_BOT_SPEED
  if  (speed > MAX_BOT_SPEED):
      speed=MAX_BOT_SPEED    
 
  vec = Vector2D()


  motionAngle = nextWP.angle(botPos)
  theta  = motionAngle - state.homePos[bot_id].theta

  if False:
      if distan < DRIBBLER_BALL_THRESH:
          if distan < BOT_BALL_THRESH:
              cmd_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,dribller)
            
          else:
              cmd_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
      else:
          cmd_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
  else:
      if distan > BOT_BALL_THRESH:
          cmd_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
      else:
          cmd_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, 0,dribller)



    # if dist < DRIBBLER_BALL_THRESH:
    #     if dist < 1*BOT_BALL_THRESH:
    #         kub.move(0,0)
    #         # kub.turn(omega)
    #         # kub.kick(7)
    #         # kub.dribble(True)
    #         # print "in 1"
    #         # cmd_node.send_command(pub, state.isteamyellow, kub.kubs_id, 0, 0, omega, 0, True)
    #     else:
    #         kub.move(speed * math.sin(-theta), speed * math.cos(-theta))
    #         # kub.turn(omega)
    #         # kub.dribble(True)
    #         # print "in 2"
    #         # cmd_node.send_command(pub, state.isteamyellow, kub.kubs_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, True)

    # else:
    #     kub.move(speed * math.sin(-theta), speed * math.cos(-theta))
    #     kub.turn(omega)
        # print "in 3"
   #print kub.vx,kub.vy
  # kub.execute()
        # cmd_node.send_command(pub, state.isteamyellow, kub.kubs_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, False)
def BS_callback(state):
	kub.update_state(state)
	GoToPoint(state = state,point = state.ballPos,kub = kub,orient_theta = 0)

def main():
	rospy.init_node('goalie',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	rospy.spin()

if __name__ == "__main__":
	main()

