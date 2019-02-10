from enum import Enum
import behavior
import _GoToPoint_
import rospy
from utils.functions import *
from utils.state_functions import *
from utils.config import *
from velocity.run import *
import _turnAround_

start_time = None


class allignTowardsPoint(behavior.Behavior):
	class State(Enum):
		normal = 1
		turnAround = 2

	def __init__(self,target,continuous=False):

		super(allignTowardsPoint,self).__init__()

		self.name = "allignTowardsPoint"

		self.power = 7.0

		self.target_point = target 

		self.go_at = None 

		#self.setStanceThresh = 1.5*BOT_RADIUS

		self.ball_dist_thresh = BOT_BALL_THRESH

		self.behavior_failed = False

		self.add_state(allignTowardsPoint.State.normal,
			behavior.Behavior.State.running)


		self.add_state(allignTowardsPoint.State.turnAround,
			behavior.Behavior.State.running)
		

		self.add_transition(behavior.Behavior.State.start,
			allignTowardsPoint.State.normal,lambda: True,'immediately')


		self.add_transition(allignTowardsPoint.State.normal,
			allignTowardsPoint.State.turnAround,lambda: not self.is_alligned(), 'near_enough')


		self.add_transition(allignTowardsPoint.State.turnAround,
			behavior.Behavior.State.completed,lambda: self.is_alligned(),'kicked!!!')

		self.add_transition(allignTowardsPoint.State.normal,
			behavior.Behavior.State.completed,lambda: self.is_alligned(),'kicked!!!')

		self.add_transition(allignTowardsPoint.State.turnAround,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

	
	def add_kub(self,kub):
		self.kub = kub

	def add_theta(self,theta):
		self.theta = theta

	def get_pos_as_vec2d(self, point2d):
		return Vector2D(int(point2d.x), int(point2d.y))

	def add_target_theta(self):
		self.target_theta = angle_diff(self.kub.state.ballpos, self.kub.get_pos())


	def turnaround(self):
		return not self.bot_moving() #and not self.facing_the_target() and self.near_targetBall_line() and self.one_more()

	def turnAroundDirect(self):
		bot_pos = Vector2D(self.kub.state.homePos[self.kub.kubs_id].x, self.kub.state.homePos[self.kub.kubs_id].y)
		ball_pos = Vector2D(self.kub.state.ballPos.x, self.kub.state.ballpos.y)

		return 

	# def GoAndKickDirect(self):
	# 	return self.facing_the_target() and not self.setstance()

	# def goandkick(self):
	# 	print("facing_the_target : ", self.facing_the_target())
	# 	return self.facing_the_target()

	def near_targetBall_line(self):
		lin = Line(point1 = self.get_pos_as_vec2d(self.kub.state.ballPos), angle = None, point2 = self.target_point)
		disst = lin.distance_from_point(self.get_pos_as_vec2d(self.kub.get_pos()))
		return disst < 0.75*BOT_RADIUS

	def one_more(self):
		dis1 = self.get_pos_as_vec2d(self.kub.get_pos()).dist(self.target_point)
		dis2 = self.get_pos_as_vec2d(self.kub.state.ballPos).dist(self.target_point)
		return dis1 > dis2

	def bot_moving(self):
		if abs(self.kub.state.homeVel[self.kub.kubs_id].x) != 0 or abs(self.kub.state.homeVel[self.kub.kubs_id].y) != 0:
			return True
		return False

	def ball_nearby(self, thresh = BOT_RADIUS*1.5):
		if bot_in_front_of_ball(self.kub, thresh):
			return True
		else :
			return False

	def facing_the_target(self):
		#print(" theta1 : ", self.theta)
		print(" bot theta : ", self.kub.get_theta())
		if vicinity_theta( self.theta, normalize_angle(self.kub.get_theta()), thresh=0.30 ):
			return True
		else :
			return False

	def at_ball_pos(self):
		error = 10
		return vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=BOT_BALL_THRESH+error)



	def reset(self):
		global start_time
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

	def execute_normal(self):
		pass	

	def on_enter_turnAround(self):
		#print("entered turnAround")
		#print(self.get_pos_as_vec2d(self.kub.get_pos()).dist(self.get_pos_as_vec2d(self.kub.state.ballPos)))
		self.theta = normalize_angle(angle_diff(self.kub.state.homePos[self.kub.kubs_id],self.target_point))
		_turnAround_.init(self.kub, self.theta )
		pass	


	def execute_turnAround(self):
		#print("executing turnaround")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _turnAround_.execute(start_time)
		for gf in generatingfunction:
			self.kub,final_theta = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			if not vicinity_theta(self.theta,final_theta,thresh=0.08):
				self.behavior_failed = True
				break

	def is_alligned(self):
		ball_pos = self.get_pos_as_vec2d(self.kub.state.ballPos)
		bot_pos = self.get_pos_as_vec2d(self.kub.state.homePos[self.kub.kubs_id])
		orientation = Vector2D(ball_pos.x - bot_pos.x, ball_pos.y - bot_pos.y)
		angle = normalize_angle(orientation.tan_inverse() - self.kub.state.homePos[self.kub.kubs_id].theta)
		#angle = normalize_angle(angle)
		if abs(angle) < 0.08:
			return True
		return False		

	def on_enter_normal(self):
		pass

	def on_exit_normal(self):
		pass

	def on_exit_turnAround(self):
		print("not exiting")
		pass



