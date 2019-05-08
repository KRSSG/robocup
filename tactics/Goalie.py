import behavior
from role import GoToPoint,GoToBall,_GoToPoint_, _turnAround_, KickToPoint
import enum
from math import atan2
from utils.functions import *
from utils.config import *
import memcache
import os
import rospy
shared = memcache.Client(['127.0.0.1:11211'],debug=False)
from krssg_ssl_msgs.srv import  bsServer


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

class Goalie(behavior.Behavior):

	MIN_VEL = 10.0
	behavior_failed = False

	class State(enum.Enum):
		## Normal gameplay, stay towards the side of the goal that the ball is on.
		protect = 1
		## Get the ball out of our defense area.
		clear = 2
		## Keep calm and wait for the ball to be valid.
		peace = 3
		## Block the shots
		block = 4

	def __init__(self,continuous=False):
		super(Goalie,self).__init__()

		for substate in Goalie.State:
			self.add_state(substate, behavior.Behavior.State.running)
		
		self.add_transition(behavior.Behavior.State.start, Goalie.State.clear, lambda: self._peace_to_clear(), "direct clear")

		self.add_transition(behavior.Behavior.State.start,
			Goalie.State.block, lambda: self._peace_to_block(), "direct block")

		self.add_transition(behavior.Behavior.State.start,
			Goalie.State.protect, lambda: self._peace_to_protect() and not self._peace_to_block(), "direct protect")
		
		self.add_transition(behavior.Behavior.State.start,
			Goalie.State.peace, lambda: self.peace_out() , "immediately")

		self.add_transition(Goalie.State.peace,Goalie.State.clear, lambda: self._peace_to_clear(),"clear now")

		self.add_transition(Goalie.State.peace,
			Goalie.State.block, lambda: self._peace_to_block(), "opponent shot")

		self.add_transition(Goalie.State.peace,
			Goalie.State.protect, lambda: self._peace_to_protect() and not self._peace_to_block(), "ball is valid")

		self.add_transition(Goalie.State.protect, Goalie.State.clear, lambda: self._protect_to_clear(),"clear now")

		self.add_transition(Goalie.State.protect, Goalie.State.block, lambda: self._peace_to_block(),"block now")

		self.add_transition(Goalie.State.protect, Goalie.State.peace, lambda: self.peace_out(),"relax")

		self.add_transition(Goalie.State.block, Goalie.State.clear, lambda: self._block_to_clear(),"clear now")

		self.add_transition(Goalie.State.block, Goalie.State.protect, lambda: self._peace_to_protect() and not self._peace_to_block,"remain vigilant")
		
		self.add_transition(Goalie.State.block, Goalie.State.peace, lambda: self.peace_out(),"relax")
		
		self.add_transition(Goalie.State.protect,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
		self.add_transition(Goalie.State.block,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
		self.add_transition(Goalie.State.peace,
		    behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
		self.add_transition(Goalie.State.clear,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

	def GoToPoint(self,point,theta):
		pass

	def outOfField(self, state):
		if state.ballPos.x < -HALF_FIELD_MAXX or state.ballPos.x > HALF_FIELD_MAXX or state.ballPos.y > HALF_FIELD_MAXY or state.ballPos.y < -HALF_FIELD_MAXY:
			return True
		return False


	def add_kub(self,kub):
		self.kub = kub

	def peace_out(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		if state.ballPos.x > 0:
			return True
		return False

	def _peace_to_clear(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		return state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX
		pass

	def _peace_to_protect(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		return  (opponent_bot_with_ball(state) != None) or ((state.ballVel.x < -self.MIN_VEL or state.ballPos.x<=0) and \
			not (state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX))

	def _peace_to_block(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		return  (opponent_bot_with_ball(state) != None) and ((state.ballPos.x < -HALF_FIELD_MAXX/2 and not state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX) or (abs(state.ballPos.y) > OUR_GOAL_MAXY and state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX))

	def _protect_to_clear(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		return state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX and state.ballVel.x < 0 and abs(state.ballPos.y) < OUR_GOAL_MAXY

	def _block_to_clear(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		return state.ballPos.x < -HALF_FIELD_MAXX + 2*OUR_GOAL_MAXX and abs(state.ballPos.y) < OUR_GOAL_MAXY

	# note that execute_running() gets called BEFORE any of the execute_SUBSTATE methods gets called

	def execute_peace(self):
		print("Sab moh maya hain")
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		if abs(state.ballPos.y) < OUR_GOAL_MAXY:
			expected_y = state.ballPos.y
		else:
			if state.ballPos.y < 0:
				expected_y = OUR_GOAL_MINY
			else:
				expected_y = OUR_GOAL_MAXY
		print("In Peace :", state.ballPos.y)
		point = Vector2D(-HALF_FIELD_MAXX + 5*BOT_RADIUS, expected_y)
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,BOT_BALL_THRESH)
		for gf in generatingfunction:
			self.kub,aim = gf
			print("peace gf", point.x + point.y)
			if not vicinity_points(aim,point,thresh=BOT_RADIUS) or self.outOfField(state):
				self.behavior_failed = True
				print("Bhenchod:")
				break


		pass


	def execute_clear(self):
		print("Clear kar raha hoon")
		target = Vector2D(4000,0)
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		self.ktp = KickToPoint.KickToPoint(target)
		self.ktp.add_kub(self.kub)
		self.ktp.add_theta(theta=normalize_angle(atan2(target.y - state.ballPos.y,target.x - state.ballPos.x)))
		self.ktp.spin()

	def execute_protect(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		print("main rakshak hoon")
		if state.ballVel.x < -10:  
			angle = state.ballVel.y/state.ballVel.x
			expected_y = angle*(-HALF_FIELD_MAXX + 3*BOT_RADIUS - state.ballPos.x) + state.ballPos.y
		else:
			if abs(state.ballPos.y) < OUR_GOAL_MAXY:
				expected_y = state.ballPos.y
			else:
				if state.ballPos.y < 0:
					expected_y = OUR_GOAL_MINY
				else:
					expected_y = OUR_GOAL_MAXY
		point = Vector2D(-HALF_FIELD_MAXX + 3*BOT_RADIUS,expected_y)
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,BOT_BALL_THRESH)
		for gf in generatingfunction:
			self.kub,aim = gf
			print("gf", point.x, point.y)
			if not vicinity_points(aim,point,thresh=BOT_RADIUS) or self.outOfField(state):
				self.behavior_failed = True
				print("Fail Fail")
				break

	def execute_block(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		print("main de Gea hoon")
		messi = opponent_bot_with_ball(state)
		angle = state.awayPos[messi].theta
		expected_y = (math.tan(angle))*(-HALF_FIELD_MAXX + BOT_RADIUS - state.ballPos.x) + state.ballPos.y
		point = Vector2D()

		if expected_y < OUR_GOAL_MINY:
			expected_y = OUR_GOAL_MINY
		if expected_y > OUR_GOAL_MAXY:
			expected_y = OUR_GOAL_MAXY
		point = Vector2D(-HALF_FIELD_MAXX + BOT_RADIUS,expected_y)
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,BOT_BALL_THRESH)
		for gf in generatingfunction:
			self.kub,aim = gf
			print("gf", point.x, point.y)
			if not vicinity_points(aim,point,thresh=BOT_RADIUS) or self.outOfField(state):
				self.behavior_failed = True
				print("Fail Fail")
				break

	def on_exit_clear(self):
		pass
	def on_exit_peace(self):
		pass
	def on_exit_protect(self):
		pass
	def on_exit_block(self):
		pass
	def on_enter_peace(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		if abs(state.ballPos.y) < OUR_GOAL_MAXY:
				expected_y = state.ballPos.y
		else:
			if state.ballPos.y < 0:
				expected_y = OUR_GOAL_MINY
			else:
				expected_y = OUR_GOAL_MAXY

		point = Vector2D(-HALF_FIELD_MAXX + 5*BOT_RADIUS, expected_y)
		theta = self.kub.get_pos().theta
		_GoToPoint_.init(self.kub, point, theta)
		pass
	def on_enter_clear(self):
		pass
	def on_enter_protect(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		#expected_y = goalie_expected_y(state, self.kub.kubs_id)
		if abs(state.ballVel.x) < 10:
			if abs(state.ballPos.y) < OUR_GOAL_MAXY:
				expected_y = state.ballPos.y
			else:
				if state.ballPos.y < 0:
					expected_y = OUR_GOAL_MINY
				else:
					expected_y = OUR_GOAL_MAXY
		else:
			angle = state.ballVel.y/state.ballVel.x
			expected_y = angle*(-HALF_FIELD_MAXX + 3*BOT_RADIUS - state.ballPos.x) + state.ballPos.y
		point = Vector2D()

		if expected_y < OUR_GOAL_MINY:
			expected_y = OUR_GOAL_MINY
		if expected_y > OUR_GOAL_MAXY:
			expected_y = OUR_GOAL_MAXY

		point = Vector2D(-HALF_FIELD_MAXX + 3*BOT_RADIUS, expected_y)
		
		print("Entered Protect , going to", point.x, point.y)
		theta = self.kub.get_pos().theta
		_GoToPoint_.init(self.kub, point, theta)
		pass
	def on_enter_block(self):
		state = None
		try:
			state = getState(state)
		except rospy.ServiceException, e:
			print e
		if state:
			state = state.stateB
		#expected_y = goalie_expected_y(state, self.kub.kubs_id)
		messi = opponent_bot_with_ball(state)
		angle = state.awayPos[messi].theta
		expected_y = (math.tan(angle))*(-HALF_FIELD_MAXX + BOT_RADIUS - state.ballPos.x) + state.ballPos.y
		point = Vector2D()

		if expected_y < OUR_GOAL_MINY:
			expected_y = OUR_GOAL_MINY
		if expected_y > OUR_GOAL_MAXY:
			expected_y = OUR_GOAL_MAXY

		point = Vector2D(-HALF_FIELD_MAXX + BOT_RADIUS, expected_y)
		
		# if expected_y > 0:
		# 	point = Vector2D(-HALF_FIELD_MAXX + 2*BOT_RADIUS,expected_y)
		# else:
		# 	point = Vector2D(-HALF_FIELD_MAXX + 2*BOT_RADIUS,expected_y)
		print("Entered Block , going to", point.x, point.y)
		theta = angle + 3.141592654
		_GoToPoint_.init(self.kub, point, theta)
		pass
