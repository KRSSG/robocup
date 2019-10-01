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
state = None
POINTPREDICTIONFACTOR = 2

kub = kubs.kubs(0,state, pub)


def GoToPoint(state,point,kub,orient_theta=None, going_to_ball = False):
	# state = None
	# 	try:
	# 		state = getState(state)
	# 	except rospy.ServiceException, e:
	# 		print e
	# 	if state:
	# 		state = state.stateB
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
	if(point.x==0):
		point.x = 1
	if(point.y==0):
		point.y = 1
	pointPos.x = int(point.x)
	pointPos.y = int(point.y)
	point = Vector2D()
	nextWP = Vector2D()
	nextNWP = Vector2D()

	pathplanner = MergeSCurve()

	botPos = Vector2D(int(state.homePos[bot_id].x), int(state.homePos[bot_id].y))
	ballPos = Vector2D(state.ballPos.x, state.ballPos.y)
	if(ballPos.x==0):
		ballPos.x = 1
	if(ballPos.y==0):
		ballPos.y = 1
	orient_theta = botPos.angle(ballPos)
	print("orient_theta = ", orient_theta)

	# pointPos.x, pointPos.y = 500, 500
	# print("pointPos = ", pointPos.x, pointPos.y)
 
	pathplanner.plan(botPos,pointPos,nextWP,nextNWP,obs,len(obs),bot_id, True)
	# print("nextNWP")
	v = Vector2D()
	distan = botPos.dist(pointPos)
	maxDisToTurn = distan 
	print("botPos = ", botPos.x, botPos.y, ", pointPos = ", pointPos.x, pointPos.y, " maxDisToTurn = ", maxDisToTurn)
	print("nextWP = ", nextWP.x, nextWP.y)
	angleToTurn = v.normalizeAngle((orient_theta)-(state.homePos[bot_id].theta))

	minReachTime = maxDisToTurn / MAX_BOT_OMEGA
	maxReachTime = maxDisToTurn / MIN_BOT_OMEGA

	minTurnTime = angleToTurn / MAX_BOT_OMEGA
	maxTurnTime = angleToTurn / MIN_BOT_OMEGA

	speed = 0.0
	MIN_BOT_OMEGA_H = MIN_BOT_OMEGA
	omega = 1*angleToTurn * MAX_BOT_OMEGA / (2 * math.pi)                 

	if abs(omega)<MIN_BOT_OMEGA_H:
			if omega < 0:
					omega = -MIN_BOT_OMEGA_H
			else:
					omega = MIN_BOT_OMEGA_H

	
	# omega = 0
	from math import exp

	speed= 3*maxDisToTurn*MAX_BOT_SPEED/(HALF_FIELD_MAXX)
	print("Speed before ", speed)
	if (speed)< 2*MIN_BOT_SPEED:
			speed=2*MIN_BOT_SPEED
	if  (speed > MAX_BOT_SPEED):
			speed=MAX_BOT_SPEED    
 
	vec = Vector2D()
	# print("speed after, ", speed)
	# print("nextWP = ", nextWP.x, nextWP.y)
	motionAngle = nextWP.angle(botPos)
	print("bOT pOS, angle =  ", botPos.x, botPos.y, state.homePos[bot_id].theta)
	theta  = motionAngle - state.homePos[bot_id].theta
	print(" maxDisToTurn = ", maxDisToTurn)
	print("MIN_BOT_OMEGA = ", MIN_BOT_OMEGA_H, " MAX_BOT_OMEGA = ", MAX_BOT_OMEGA)
	# print("Theta ", theta)

	if False:
			if distan < DRIBBLER_BALL_THRESH:
					if distan < BOT_BALL_THRESH:
							print("GOOOOOOOO")
							ball_vel = Vector2D(state.homePos[bot_id].x - state.balLPos.x, state.homePos[bot_id].y - state.balLPos.y)
							angle_ball_vel = ball_vel.tan_inverse()
							cmd_node.send_command(pub, state.isteamyellow, bot_id, MAX_BOT_SPEED*0.2 * math.cos(angle_ball_vel),MAX_BOT_SPEED*0.2 * math.sin(angle_ball_vel) ,omega, 0,dribller)
						
					else:
							cmd_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
			else:
					cmd_node.send_command(pub, state.isteamyellow, bot_id, speed * math.sin(-theta), speed * math.cos(-theta), omega, 0, dribller)
	else:
			if distan > BOT_BALL_THRESH:
					vx, vy = speed * math.sin(-theta), speed * math.cos(-theta)
					cmd_node.send_command(pub, state.isteamyellow, bot_id, -vy, vx, omega, 0, dribller)
			else:
					kick_power = 0
					dribller = 0
					if(going_to_ball):
						dribller = 1
						kick_power = 7
					cmd_node.send_command(pub, state.isteamyellow, bot_id, 0, 0, omega, kick_power,dribller)


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
D_BOX_Y = 600
X_GOAL_THRESH = -1400
HALF_FIELD_MAXX = 2600
def BS_callback(state):
	kub.update_state(state)
	print("BYYY")
	if(state.ballPos.x<X_GOAL_THRESH and abs(state.ballPos.y)<D_BOX_Y):
		point = Vector2D(state.ballPos.x - 200,state.ballPos.y - 200)
		#point.y = state.ballPos.y - 200
	 	GoToPoint(state = state,point = point,kub = kub,orient_theta = 0, going_to_ball = True)
	else:
		pointT = Vector2D(-2450, state.ballPos.y)
		if(abs(pointT.y)>D_BOX_Y):
			pointT.y = pointT.y/abs(pointT.y)*D_BOX_Y
		GoToPoint(state = state,point = pointT,kub = kub,orient_theta = 0)
def main():
	print("hello")
	rospy.init_node('goalie',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	rospy.spin()

if __name__ == "__main__":
	main()

