from enum import Enum
import behavior
import _GoToPoint_
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
center_arc=Vector2D(-HALF_FIELD_MAXX,0)
point=Vector2D((-HALF_FIELD_MAXX + DBOX_HEIGHT)+BOT_RADIUS , 0.5*DBOX_WIDTH)
class defender(behavior.Behavior):

	class State(Enum):
		setup = 1
		block = 2
		intercept = 3
		clear = 4

	def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

		super(defender,self).__init__()
		self.course_approch_thresh = course_approch_thresh
		self.name = "Square"
		self.kubs = []
		self.blocker = None
		self.intercepter = None
		self.intercepting_point = Vector2D()
		self.pass_angle = None
		self.behavior_failed = False
		self.ball_kicked = False
		self.kicking_point=Vector2D(0,0)
		self.line = Line(point1=Vector2D((-HALF_FIELD_MAXX + DBOX_HEIGHT)+5*BOT_RADIUS,0),angle=math.pi/2.0)
		self.line1= Line(point1=Vector2D(0,-0.5*DBOX_WIDTH-5*BOT_RADIUS),angle=0)
		self.line2= Line(point1=Vector2D(0,0.5*DBOX_WIDTH+5*BOT_RADIUS),angle=0)
		self.goal_line= Line(point1=Vector2D(-HALF_FIELD_MAXX,0),angle=math.pi/2.0)
		for state in defender.State:
			self.add_state(state,behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,defender.State.setup,
			lambda:True,"immediately")
		self.add_transition(defender.State.setup, defender.State.clear,
			lambda:self.ready_to_kick(),"Direct kick")
		self.add_transition(defender.State.setup,defender.State.intercept,
			lambda:self.should_intercept(),"passer ready")
		self.add_transition(defender.State.setup,defender.State.block,
			lambda:not self.should_intercept() ,"threat")
		self.add_transition(defender.State.clear,defender.State.intercept,
			lambda:self.should_intercept() and not self.ready_to_kick(),"kicking")
		self.add_transition(defender.State.clear,defender.State.block,
			lambda:not self.ready_to_kick() or self.bot_infrontof_ball(),"ball kicked")
		self.add_transition(defender.State.block,defender.State.clear,
			lambda:self.ready_to_kick(),"receiver found")
		self.add_transition(defender.State.block,defender.State.intercept,
			lambda:self.should_intercept(),"moving to receiving point")
		self.add_transition(defender.State.intercept,defender.State.clear,
			lambda:self.ready_to_kick(),"ball incoming")
		self.add_transition(defender.State.intercept,defender.State.block,
			lambda:not self.should_intercept(),"ball received")


		for state in defender.State:
			self.add_transition(state,defender.State.setup,
				lambda:self.behavior_failed,"failed")


	def add_kub(self,kub):
		self.kub = kub

	def add_theta(self,theta):
		self.theta = theta

	def ready_to_kick(self):
		return vicinity_points(getPointBehindTheBall(Vector2D(self.kub.state.ballPos),0), Vector2D(self.kub.get_pos()), thresh = 4*BOT_RADIUS)
	
	def ball_moving(self):
		if magnitute(self.kub.state.ballVel) <= 50:
			return False
		return True

	def bot_infrontof_ball(self):
		if(self.kub.get_pos().x>self.kub.state.ballPos.x):
			return True
		return False


	def on_enter_setup(self):
		self.behavior_failed=False
		pass

	def execute_setup(self):
		pass

	def on_exit_setup(self):
		pass

	def on_enter_block(self):
		pass

	def execute_block(self):
		ball=Vector2D(self.kub.state.ballPos)
		goal_line=Line(point1=Vector2D(self.kub.state.ballPos),point2=Vector2D(-HALF_FIELD_MAXX,0))
		target_point=self.line.intersection_with_line(goal_line)
		target_point.y+=BOT_RADIUS
		if target_point.y > 0.5*DBOX_WIDTH+5*BOT_RADIUS:
			target_point=self.line2.intersection_with_line(goal_line)
		if target_point.y < -0.5*DBOX_WIDTH-5*BOT_RADIUS:
			target_point=self.line1.intersection_with_line(goal_line)
		if ball_moving_towards_our_goal(self.kub.state):
			ball_line=Line(point1=Vector2D(self.kub.state.ballPos),angle=math.atan2(self.kub.state.ballVel.y,self.kub.state.ballVel.x))
			point=ball_line.intersection_with_line(self.goal_line)
			if point.y*point.y<0.25*DBOX_WIDTH*DBOX_WIDTH :
				target_point=self.line.intersection_with_line(ball_line)
				if target_point.y > 0.5*DBOX_WIDTH+5*BOT_RADIUS:
					target_point=self.line2.intersection_with_line(ball_line)
				if target_point.y < -0.5*DBOX_WIDTH-5*BOT_RADIUS:
					target_point=self.line1.intersection_with_line(ball_line)

		_GoToPoint_.init(self.kub, target_point, 0)
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
		#self.behavior_failed = False
		prev_point=Vector2D(self.kub.state.ballPos)
		next_point = prev_point
		movement=self.kub.get_pos()
		point=Vector2D()
		for gf in generatingfunction:
			# print (target_point.x)
			next_point = Vector2D(self.kub.state.ballPos)
			self.kub,point = gf
			if dist(next_point,prev_point) > 3*BOT_RADIUS:
				break
			if not vicinity_points(point,target_point,thresh=BOT_RADIUS*2.0):
				break
			if self.should_intercept():
				break
			if self.ready_to_kick():
				break 	
			prev_point=next_point

	def on_exit_block(self):
		pass

	def intercept_complete(self):
		ball_vel = Vector2D(self.kub.state.ballVel.y,self.kub.state.ballVel.x)
		ball_vel_angle = ball_vel.tan_inverse()
		bot_ball_dir = Vector2D(self.kub.state.ballPos.y-self.kub.state.homePos[self.kub.kubs_id].y , self.kub.state.ballPos.x-self.kub.state.homePos[self.kub.kubs_id].x)
		if ( abs(ball_vel_angle - bot_ball_dir.tan_inverse() )< 0.0523599):
			return 1
		return 0

	def should_intercept(self):
		ball=Vector2D(self.kub.state.ballPos)
		if not self.ball_moving:
			ball_line=Line(point1=ball,angle=math.atan2(self.kub.state.ballVel.y,self.kub.state.ballVel.x))
		ball_path_point=ball
		if not self.ball_moving():
			opp_time=1000000000000000
			for opp_id, awayPos in enumerate(self.kub.state.awayPos):
				if dist(awayPos,getPointBehindTheBall(ball,math.pi))/MAX_BOT_SPEED<opp_time:
					opp_time=dist(awayPos,getPointBehindTheBall(ball,math.pi))/MAX_BOT_SPEED
			our_time=dist(self.kub.state.homePos[1],getPointBehindTheBall(ball,0))/MAX_BOT_SPEED
			if our_time > dist(self.kub.state.homePos[2],getPointBehindTheBall(ball,0))/MAX_BOT_SPEED:
				return False
			if our_time < 0.8*opp_time:
				self.intercepting_point=getPointBehindTheBall(ball,0)
				return True
		# for x in range(1, 31):
		# 	ball_path_point.x=ball_path_point.x+DBOX_HEIGHT*0.2*x*math.cos(ball_line.angle)
		# 	ball_path_point.y=ball_path_point.y+DBOX_HEIGHT*0.2*x*math.cos(ball_line.angle)
		# 	ball_time=abs(dist(ball_path_point,ball)/magnitute(self.intercepter.state.ballVel))
		# 	opp_time=1000000000000000
		# 	for opp_id, awayPos in enumerate(self.kubs[0].state.awayPos)
		# 		if dist(awayPos,getPointBehindTheBall(ball_path_point,math.pi))/MAX_BOT_SPEED<opp_time:
		# 			opp_time=dist(awayPos,getPointBehindTheBall(ball_path_point,math.pi))/MAX_BOT_SPEED
		# 	our_time=findPath_time(self.intercepter.get_pos(),getPointBehindTheBall(ball_path_point,0))
		# 	if our_time<opp_time and our_time<0.9*ball_time:
		# 		self.intercepting_point=getPointBehindTheBall(ball_path_point,0)
		#	return True   		

	def on_enter_intercept(self):
		angle=angle_diff(self.kub.get_pos(),self.intercepting_point)
		_GoToPoint_.init(self.kub, self.intercepting_point,angle)

	def execute_intercept(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
		point=self.intercepting_point
		for gf in generatingfunction:
			self.kub,point = gf
			if not vicinity_points(point,self.intercepting_point,thresh=BOT_RADIUS*2.0):
				break
			if not self.should_intercept():
				break
			if self.ready_to_kick() and not self.bot_infrontof_ball():
				break

	def on_exit_intercept(self):
		pass        

	def disable_kick(self):
		self.power = 0.0

	def on_enter_clear(self):
		print("Enter kicking")
		self.kick_power=2*math.sqrt(dist(Vector2D(DBOX_HEIGHT,0),self.kub.state.ballPos)/6400.0)*5
		angle=angle_diff(self.kub.get_pos(),self.kub.state.ballPos)
		_GoToPoint_.init(self.kub,Vector2D(self.kub.state.ballPos),angle)
		pass

	def execute_clear(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
		generatingfunction = _GoToPoint_.execute(start_time,BOT_RADIUS)
		for gf in generatingfunction:
			self.kub,point = gf
			self.kub.kick(self.kick_power)
			if not vicinity_points(point,Vector2D(self.kub.state.ballPos),thresh=BOT_RADIUS*2.0):
				break
			if not vicinity_points(self.kub.state.ballPos,self.kub.get_pos(),thresh=4*BOT_RADIUS):
				self.kick_power = 0
				break

	def on_exit_clear(self):
		print("exiting kicking")
		self.kub.reset()
		self.kub.execute()
		pass