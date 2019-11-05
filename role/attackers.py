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

class attacker(behavior.Behavior):

	class State(Enum):
		setup = 1
		defending=2
		kicking=3
		receiving=4

	def __init__(self,id_1,id_2,course_approch_thresh = DISTANCE_THRESH/3):
		super(attacker,self).__init__()
		self.name="attacker"+str(id_1)
		self.attacker_id=id_1
		self.team_mate_id=id_2
		self.behavior_failed=False
		self.course_approch_thresh=course_approch_thresh
		self.target_point=Vector2D()
		self.scoring_point1=Vector2D((HALF_FIELD_MAXX-DBOX_HEIGHT)*0.9,0.5*DBOX_WIDTH*0.8)
		self.scoring_point2=Vector2D((HALF_FIELD_MAXX-DBOX_HEIGHT)*0.9,-0.5*DBOX_WIDTH*0.8)
		for state in attacker.State:
			self.add_state(state,behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,attacker.State.setup, lambda: True,"immediately")


		self.add_transition(attacker.State.setup,attacker.State.defending, lambda:not self.either_has_ball(),"set to collecting")
		self.add_transition(attacker.State.setup,attacker.State.kicking, lambda: self.has_ball(),"set to collecting")
		self.add_transition(attacker.State.setup,attacker.State.receiving, lambda: not self.has_ball() and self.either_has_ball() ,"set to collecting")
		
		self.add_transition(attacker.State.defending,attacker.State.kicking, lambda: self.has_ball(),"dribbling")
		self.add_transition(attacker.State.defending,attacker.State.receiving, lambda: not self.has_ball() and self.either_has_ball(),"pass_dribble")

		self.add_transition(attacker.State.kicking,attacker.State.receiving, lambda: not self.has_ball() and self.either_has_ball(), "dribble_collect")
		self.add_transition(attacker.State.kicking,attacker.State.defending, lambda: self.opponent_has_ball(),"pass_collect")

		self.add_transition(attacker.State.receiving,attacker.State.kicking, lambda: self.has_ball(),"dribbling")
		self.add_transition(attacker.State.receiving,attacker.State.defending, lambda: self.opponent_has_ball(),"pass_dribble")

		for state in attacker.State:
			self.add_transition(state,attacker.State.setup,
				lambda:self.behavior_failed,"failed")

	def add_kub(self,kub):
		self.kub=kub

	def has_ball(self):
		if dist(self.kub.get_pos(),self.kub.state.ballPos) <1.5*BOT_RADIUS:
			return True
		else:
			return False

	def either_has_ball(self):
		if dist(self.kub.state.homePos[self.attacker_id],self.kub.state.ballPos)<1.5*BOT_RADIUS or dist(self.kub.state.homePos[self.team_mate_id],self.kub.state.ballPos)<1.5*BOT_RADIUS:
			return True
		else:
			return False

	def opponent_has_ball(self):
		flag=0
		for i in self.kub.state.awayPos:
			if dist(i,self.kub.state.ballPos)<1.5*BOT_RADIUS:
				flag=1
				break
		if flag:
			return True
		else:
			return False


	def on_enter_setup(self):
		pass
	def execute_setup(self):
		pass
	def on_exit_setup(self):
		pass


 	def get(self):
 		if(dist(self.kub.state.homePos[self.team_mate_id],self.kub.state.ballPos)<dist(self.kub.state.homePos[self.attacker_id],self.kub.state.ballPos)):
 			if(dist(self.kub.state.homePos[self.attacker_id],self.scoring_point2)<dist(self.kub.state.homePos[self.attacker_id],self.scoring_point1)):
 				self.target_point=self.scoring_point2
 			else:
 				self.target_point=self.scoring_point1
 		else:	
 			self.target_point=self.kub.state.ballPos
 		return self.target_point

	def on_enter_defending(self):
		self.target_point=self.get()
		theta = normalize_angle(angle_diff(self.target_point,self.kub.state.ballPos))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_defending(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,False)
		for gf in generatingfunction:
			self.kub,target_point = gf
			if self.either_has_ball():
				break
			if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*1.0):
				self.behavior_failed = True
				break

	def on_exit_defending(self):
		pass

	def get_kick_point(self):
		if self.kub.get_pos().x>self.kub.state.homePos[self.team_mate_id].x and self.kub.get_pos().x>0:
			return self.kub.state.ballPos
		else:
			return Vector2D(HALF_FIELD_MAXX,0)

	def on_enter_kicking(self):
		self.target_point=self.get_kick_point()
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

	def on_enter_receiving(self):
		goal_point=Vector2D(HALF_FIELD_MAXX,OUR_GOAL_MINY)
		angle = 0.75*angle_diff(self.kub.state.ballPos,self.kub.get_pos())+0.25*angle_diff(goal_point,self.kub.get_pos())
		_turnAround_.init(self.kub,angle)
		pass

	def execute_receiving(self):
		angle = angle_diff(self.kub.state.ballPos,self.kub.get_pos())
		if not vicinity_theta(self.kub.get_theta(),angle,thresh=0.05):
			start_time = rospy.Time.now()
			start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
			generatingfunction = _turnAround_.execute(start_time,0.1)
			for gf in generatingfunction:
				angle = angle_diff(self.kub.state.ballPos,self.kub.get_pos())
				self.kub,final_theta = gf
				# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
				if self.opponent_has_ball():
					break
				if self.has_ball() and vicinity_theta(angle,final_theta,thresh=0.1):
					break
				if not vicinity_theta(angle,final_theta,thresh=0.1):
					self.behavior_failed = True
					break
				

	def on_exit_receiving(self):
		pass

