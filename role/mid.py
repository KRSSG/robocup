from enum import Enum
import behavior
import _GoToPoint_ , _turnAround_ , KickToPoint
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from velocity.run import *
import time
from krssg_ssl_msgs.srv import *

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

class mid(behavior.Behavior):

	class State(Enum):
		setup = 1
		defending=2
		kicking=3

	def __init__(self,id1,course_approch_thresh = DISTANCE_THRESH/3):
		super(mid,self).__init__()
		self.name="mid"
		self.id=id1
		self.behavior_failed=False
		self.course_approch_thresh=course_approch_thresh
		self.target_point=Vector2D()

		for state in mid.State:
			self.add_state(state,behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,mid.State.setup, lambda: True,"immediately")


		self.add_transition(mid.State.setup,mid.State.defending, lambda:not self.has_ball(),"set to collecting")
		self.add_transition(mid.State.setup,mid.State.kicking, lambda: self.has_ball(),"set to collecting")
		
		self.add_transition(mid.State.defending,mid.State.kicking, lambda: self.has_ball(),"dribbling")		
		self.add_transition(mid.State.kicking,mid.State.defending, lambda: not self.has_ball(),"pass_collect")

		for state in mid.State:
			self.add_transition(state,mid.State.setup,
				lambda:self.behavior_failed,"failed")

	def add_kub(self,kub):
		self.kub=kub

	def has_ball(self):
		if dist(self.kub.get_pos(),self.kub.state.ballPos) <1.5*BOT_RADIUS:
			return True
		else:
			return False

	def opponent_has_ball(self):
		flag=-1
		for i in self.kub.state.awayPos:
			if dist(i,self.kub.state.ballPos)<1.5*BOT_RADIUS:
				flag=i
				break
		return flag

	def opponent_closest_to_goal(self):
		flag=-1
		x=1000000
		for i in self.kub.state.awayPos:
			if(i.x)<x:
				flag=i
				x=i.x
		return flag

	def on_enter_setup(self):
		pass
	def execute_setup(self):
		pass
	def on_exit_setup(self):
		pass

	def on_enter_defending(self):
		point1=Vector2D(self.opponent_closest_to_goal())
		point2=self.kub.state.ballPos
		self.target_point=Vector2D(0.9*point1.x+0.1*point2.x,0.9*point1.y+0.1*point2.y)
		theta = normalize_angle(angle_diff(self.target_point,self.kub.state.ballPos))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_defending(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,False)
		for gf in generatingfunction:
			self.kub,target_point = gf
			if self.has_ball():
				break
			if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*1.0):
				self.behavior_failed = True
				break

	def on_exit_defending(self):
		pass

	def get_kick_point(self):
		flag=-1
		x=-1000000
		for i in self.kub.state.homePos:
			if(i.x)>x and i!=self.id:
				flag=i
				x=i.x
		return flag

	def on_enter_kicking(self):
		self.target_point=Vector2D(self.get_kick_point())
		theta = normalize_angle(angle_diff(self.kub.get_pos(),self.target_point))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_kicking(self):
		g_fsm = KickToPoint.KickToPoint(self.target_point)
		self.kick_angle=angle_diff(self.kub.state.ballPos,self.target_point)
		state = None
		state = getState(state)
		state = state.stateB
		self.kub.update_state(state)
		g_fsm.add_kub(self.kub)
		g_fsm.add_theta(self.kick_angle)
		print("***************************** KICKING **************************")
		g_fsm.spin()
		if g_fsm.state == behavior.Behavior.State.completed:
			self.ball_kicked = True
		else:
			self.behavior_failed = True

	def on_exit_kicking(self):
		print("exiting kicking")
		self.kub.reset()
		self.kub.execute()
		pass

