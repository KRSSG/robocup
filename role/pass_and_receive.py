import sys,rospy
import composite_behavior
import behavior
from utils.config import *
import time
from enum import Enum
import logging
from role import pass_receive
from role import GoToBall, GoToPoint, KickToPoint, _GoToPoint_,_turnAround_
from utils.geometry import *
from utils.functions import *
from krssg_ssl_msgs.srv import bsServer
import math

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

class PassReceive(behavior.Behavior):

	class State(Enum):

		setup = 1
		passer = 2
		waiting = 3
		kicking = 4
		receiver = 5
		going_to_receiving_point = 6
		receiving = 7
		received = 8

	def __init__(self):
		super(PassReceive,self).__init__()

		self.kubs = []
		self.passer = None
		self.receiver = None
		self.receiving_point = None
		self.pass_angle = None
		self.behavior_failed = False
		self.ball_kicked = False

		for state in PassReceive.State:
			self.add_state(state,behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,PassReceive.State.setup,
			lambda:True,"immediately")
		self.add_transition(PassReceive.State.setup, PassReceive.State.kicking,
			lambda:self.passer_present() and self.kub_0_is_passer() and self.ready_to_kick(),"Direct kick")
		self.add_transition(PassReceive.State.setup,PassReceive.State.passer,
			lambda:self.passer_present() and self.kub_0_is_passer() and not self.ready_to_kick(),"passer found")
		self.add_transition(PassReceive.State.passer,PassReceive.State.waiting,
			lambda:self.receiver_present(),"passer ready")
		self.add_transition(PassReceive.State.waiting,PassReceive.State.kicking,
			lambda:self.receiver_is_ready(),"kicking")
		self.add_transition(PassReceive.State.kicking,PassReceive.State.receiving,
			lambda:self.ball_kicked,"ball kicked")
		self.add_transition(PassReceive.State.setup,PassReceive.State.receiver,
			lambda:self.receiver_present() and self.kub_0_is_receiver(),"receiver found")
		self.add_transition(PassReceive.State.receiver,PassReceive.State.going_to_receiving_point,
			lambda:self.ball_moving_to_receive_point(),"moving to receiving point")
		self.add_transition(PassReceive.State.going_to_receiving_point,PassReceive.State.receiving,
			lambda:self.reciever_at_receiving_point(),"ball incoming")
		self.add_transition(PassReceive.State.receiving,PassReceive.State.received,
			lambda:self.ball_with_receiver() or not self.ball_moving(),"ball received")
		self.add_transition(PassReceive.State.received,behavior.Behavior.State.completed,
			lambda:True,"one pass completed")

		for state in PassReceive.State:
			self.add_transition(state,behavior.Behavior.State.failed,
				lambda:self.behavior_failed,"failed")

	def passer_present(self):
		return self.passer != None

	def receiver_present(self):
		return self.receiver != None

	def kub_0_is_passer(self):
		return self.passer.kubs_id == self.kubs[0].kubs_id

	def kub_0_is_receiver(self):
		return self.receiver.kubs_id == self.kubs[0].kubs_id		

	def receiver_is_ready(self):
		return math.fabs(math.fabs(normalize_angle(self.receiver.get_theta()-self.pass_angle))-math.pi) < deg_2_radian(10.0)

	def ready_to_kick(self):
		return vicinity_points(getPointBehindTheBall(Vector2D(self.passer.state.ballPos),self.pass_angle), Vector2D(self.passer.get_pos()), thresh = 2*BOT_RADIUS)

	def ball_moving_to_receive_point(self):
		if self.ball_moving():
			ball_vel = Vector2D()
			ball_vel.x = self.receiver.state.ballVel.x
			ball_vel.y = self.receiver.state.ballVel.y
			return math.fabs(ball_vel.angle()-self.pass_angle) < math.pi/2.0
		return False

	def ball_with_receiver(self):
		return self.receiver.has_ball()

	def reciever_at_receiving_point(self):
		if vicinity_points(self.receiver.get_pos(),self.receiving_point,0.5*BOT_RADIUS):
			return True
		return False

	def ball_moving(self):
		if magnitute(self.receiver.state.ballVel) <= 50 or vicinity_points(self.receiver.state.homePos[self.passer.kubs_id],self.receiver.state.ballPos,200):
			return False
		return True

	def add_kub(self,kub):
		self.kubs += [kub]

	def kicking_power(self):
		return math.sqrt(dist(self.receiving_point,self.passer.state.ballPos)/6400.0)*5.0

	def calculate_receiving_point(self):
		ball_pos = Vector2D(self.receiver.state.ballPos.x,self.receiver.state.ballPos.y)
		ball_vel = Vector2D(self.receiver.state.ballVel.x,self.receiver.state.ballVel.y)
		self.pass_line = Line(point1=ball_pos,angle=ball_vel.angle())
		receiver_pos = Vector2D(self.receiver.get_pos().x,self.receiver.get_pos().y)
		line2 = Line(point1=receiver_pos,angle = normalize_angle(ball_vel.angle()+math.pi/2.0))
		# print(magnitute(ball_vel))
		return self.pass_line.intersection_with_line(line2)

	def on_enter_setup(self):
		#print("Entering setup")
		pass

	def execute_setup(self):
		if len(self.kubs) != 2:
			self.behavior_failed = True
			return
		if dist(self.kubs[0].state.ballPos,self.kubs[0].get_pos())< dist(self.kubs[0].state.ballPos,self.kubs[1].get_pos()):
			self.passer = self.kubs[0]
			self.receiver = self.kubs[1]
			self.receiving_point = Vector2D(self.kubs[1].get_pos())
		else:
			self.passer = self.kubs[1]
			self.receiver = self.kubs[0]
			self.receiving_point = Vector2D(self.kubs[0].get_pos())
		self.pass_angle = angle_diff(self.receiver.state.ballPos,self.receiving_point)
		# self.pass_angle = normalize_angle(self.pass_angle)
		state = None
		state = getState(state)
		state = state.stateB
		self.passer.update_state(state)
		self.receiver.update_state(state)
		print("receiver id is",self.receiver.kubs_id)

	def on_exit_setup(self):
		#print("exiting setup")
		pass

	def on_enter_passer(self):
		print("Entering passer")
		# print("#############################   GOING TO POINT ###################################")
		target_point = getPointBehindTheBall(Vector2D(self.passer.state.ballPos),normalize_angle(self.pass_angle+math.pi))
		_GoToPoint_.init(self.passer,target_point,self.passer.get_theta())
		pass

	def execute_passer(self):
		if vicinity_points(Vector2D(self.passer.get_pos()),Vector2D(self.passer.state.ballPos),4.5*BOT_RADIUS):
			pass
		# target_point = getPointBehindTheBall(self.passer.state.ballPos,normalize_angle(self.pass_angle+math.pi))
		# g_fsm = GoToPoint.GoToPoint()
		# g_fsm.add_kub(self.passer)
		# g_fsm.add_point(target_point,self.pass_angle)
		# g_fsm.avoid_ball = True
		# g_fsm.spin()
		# self.behavior_failed = g_fsm.behavior_failed
		print(self.passer.kubs_id)
		if self.ready_to_kick():
			pass
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
		generatingfunction = _GoToPoint_.execute(start_time,BOT_RADIUS,avoid_ball=True)
		for gf in generatingfunction:
			self.passer,target_point = gf
			if not vicinity_points(target_point,getPointBehindTheBall(Vector2D(self.passer.state.ballPos),normalize_angle(self.pass_angle+math.pi)),BOT_RADIUS):
				self.behavior_failed = True
				break

	def on_exit_passer(self):
		print("exiting passer")
		pass

	def on_enter_waiting(self):
		print("Enter waiting")
		self.pass_angle = angle_diff(self.receiver.state.ballPos,self.receiver.get_pos())
		_turnAround_.init(self.passer,self.pass_angle)
		pass

	def execute_waiting(self):
		# print("/////////////////////// waiting //////////////////")
		if not vicinity_theta(self.passer.get_theta(),self.pass_angle,thresh=0.05):
			start_time = rospy.Time.now()
			start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
			generatingfunction = _turnAround_.execute(start_time,0.1)
			for gf in generatingfunction:
				self.passer,final_theta = gf
				# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
				if not vicinity_theta(self.pass_angle,final_theta,thresh=0.1):
					self.behavior_failed = True
					break
		# print("[[[[[[[[[[[[[[[[[[[[[[[[[[[oriented]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")
		print("in waiting, oriented")
		while True:
			state = None
			state = getState(state)
			state = state.stateB
			self.passer.update_state(state)
			self.receiver.update_state(state)
			if self.receiver_is_ready():
				break

	def on_exit_waiting(self):
		print("exiting waiting")
		pass

	def on_enter_kicking(self):
		print("Enter kicking")
		self.kick_power = self.kicking_power()
		_GoToPoint_.init(self.passer,Vector2D(self.passer.state.ballPos),self.passer.get_theta())
		pass

	def execute_kicking(self):
		# g_fsm = KickToPoint.KickToPoint(self.receiving_point)
		# g_fsm.power = self.kick_power
		# state = None
		# state = getState(state)
		# state = state.stateB
		# self.passer.update_state(state)
		# self.receiver.update_state(state)
		# g_fsm.add_kub(self.passer)
		# g_fsm.add_theta(self.pass_angle)
		# print("***************************** KICKING **************************")
		# g_fsm.spin()
		# # self.behavior_failed = g_fsm.behavior_failed
		# if g_fsm.state == behavior.Behavior.State.completed:
		# 	self.ball_kicked = True
		# else:
		# 	self.behavior_failed = True
		print(self.kicking_power())
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
		generatingfunction = _GoToPoint_.execute(start_time,BOT_RADIUS)
		for gf in generatingfunction:
			self.passer,ballPos = gf
			self.passer.kick(self.kicking_power())
			if not vicinity_points(ballPos,Vector2D(self.passer.state.ballPos),thresh=BOT_RADIUS):
				self.behavior_failed = True
				break

	def on_exit_kicking(self):
		print("exiting kicking")
		self.passer.reset()
		self.passer.execute()
		# # if not self.behavior_failed:
		# self.ball_kicked = True
		# # else:
		# # 	print("noooooooooooooooooooooooooooooooooooooooooooooooooooooo!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		pass

	def on_enter_receiver(self):
		print("Entering receiver")
		_turnAround_.init(self.receiver,normalize_angle(self.pass_angle+math.pi))
		pass

	def execute_receiver(self):
		# target_point = Vector2D(self.receiver.get_pos().x,self.receiver.get_pos().y)
		# g_fsm = GoToPoint.GoToPoint()
		# g_fsm.add_kub(self.receiver)
		# g_fsm.add_point(target_point,normalize_angle(self.pass_angle+math.pi))
		# g_fsm.spin()
		# self.behavior_failed = g_fsm.behavior_failed
		print(self.receiver.kubs_id)
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _turnAround_.execute(start_time,0.1)
		for gf in generatingfunction:
			self.receiver,final_theta = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			if not vicinity_theta(angle_diff(Vector2D(self.receiver.get_pos()),Vector2D(self.receiver.state.ballPos)),final_theta,thresh=0.1):
				self.behavior_failed = True
				break

	def on_exit_receiver(self):
		print("exiting receiver")
		pass

	def on_enter_going_to_receiving_point(self):
		print("going to receiving point")
		if not vicinity_points(self.receiving_point,self.calculate_receiving_point(),thresh=0.5*BOT_RADIUS):
			self.receiving_point = self.calculate_receiving_point()
		_GoToPoint_.init(self.receiver,self.receiving_point,self.receiver.get_theta())
		pass

	def execute_going_to_receiving_point(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
		generatingfunction = _GoToPoint_.execute(start_time,BOT_RADIUS)
		for gf in generatingfunction:
			self.receiver,receiving_point = gf
			if not vicinity_points(self.calculate_receiving_point(),receiving_point,BOT_RADIUS):
				self.behavior_failed = True
				break

		# g_fsm = GoToPoint.GoToPoint()
		# state = None
		# state = getState(state)
		# state = state.stateB
		# self.passer.update_state(state)
		# self.receiver.update_state(state)
		# g_fsm.add_kub(self.receiver)
		# g_fsm.add_point(self.receiving_point,self.receiver.get_theta())
		# print(self.receiving_point.x, self.receiving_point.y)
		# g_fsm.spin()
		# self.behavior_failed = g_fsm.behavior_failed

		# while True:
		# 	state = None
		# 	state = getState(state)
		# 	state = state.stateB
		# 	self.passer.update_state(state)
		# 	self.receiver.update_state(state)
		# 	self.receiver.dribble = True
		# 	self.receiver.execute()
		# 	if self.receiver.has_ball():
		# 		break

	def on_exit_going_to_receiving_point(self):
		print("reached receiving point")
		pass

	def on_enter_receiving(self):
		print("entered receiving")
		pass

	def execute_receiving(self):
		while True:
			state = None
			state = getState(state).stateB
			self.receiver.update_state(state)
			self.receiver.dribble(True)
			self.receiver.execute()
			if self.ball_with_receiver() or not self.ball_moving():
				break

	def on_exit_receiving(self):
		print("exiting receiving")
		pass

	def on_enter_received(self):
		print("Entered received")
		self.receiver.reset()
		pass

	def execute_received(self):
		self.receiver.execute()

	def on_exit_received(self):
		print("exiting received")
		pass
		






