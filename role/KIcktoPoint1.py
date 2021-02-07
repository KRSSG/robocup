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
DIST_THRESH = 75

class KickToPoint(behavior.Behavior):
	"""docstring for GoToBall"""
	class State(Enum):
		normal = 1
		setStance = 2
		GoAndKick = 3

	def __init__(self,target,continuous=False):

		super(KickToPoint,self).__init__()

		self.name = "KickToPoint"

		self.power = 7.0

		self.target_point = target 

		self.go_at = None 
		self.goto_kick=False;

		self.setStanceThresh = 1.5*BOT_RADIUS

		self.ball_dist_thresh = BOT_BALL_THRESH

		self.behavior_failed = False

		self.add_state(KickToPoint.State.normal,
			behavior.Behavior.State.running)

		self.add_state(KickToPoint.State.setStance,
			behavior.Behavior.State.running)
		
		self.add_state(KickToPoint.State.GoAndKick,
			behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
			KickToPoint.State.normal,lambda: True,'immediately')

		self.add_transition(KickToPoint.State.normal,
			KickToPoint.State.setStance,lambda: self.setstance(),'far_away')

		self.add_transition(KickToPoint.State.normal,
			KickToPoint.State.GoAndKick,lambda: self.GoAndKickDirect(),'very_close')

		self.add_transition(KickToPoint.State.GoAndKick,
			behavior.Behavior.State.completed,lambda: self.at_ball_pos(),'kicked!!!')

		self.add_transition(KickToPoint.State.setStance,
			KickToPoint.State.GoAndKick,lambda: self.goto_kick,'kicking')

		self.add_transition(KickToPoint.State.setStance,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

		self.add_transition(KickToPoint.State.GoAndKick,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
	
	def add_kub(self,kub):
		self.kub = kub

	def add_theta(self,theta):
		self.theta = theta

	def get_pos_as_vec2d(self, point2d):
		return Vector2D(int(point2d.x), int(point2d.y))

	def add_target_theta(self):
		self.target_theta = angle_diff(self.kub.state.ballpos, self.kub.get_pos())

	def setstance(self):
		global DIST_THRESH
		#print("hello")
		theta = angle_diff(self.kub.state.ballPos,self.target_point)
		go_at = getPointToGo(self.kub.state.ballPos, theta) 
		print "##########################"
		return go_at.dist(self.get_pos_as_vec2d(self.kub.get_pos())) >= DIST_THRESH*1.00


	def GoAndKickDirect(self):
		return not self.setstance()

	def goandkick(self):
		print("facing_the_target : ", self.facing_the_target())
		return self.facing_the_target()

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
		print(" theta1 : ", self.theta)
		print(" bot theta : ", self.kub.get_theta())
		if vicinity_theta( self.theta, self.kub.get_theta(), thresh=0.30 ):
			return True
		else :
			return False

	def at_ball_pos(self):
		error = 10
		return vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=BOT_BALL_THRESH+error)


	# def on_enter_setStance(self):
	# 	global start_time
	# 	start_time = rospy.Time.now()
	# 	start_time = start_time.secs + 1.0*start_time.nsecs/pow(10,9)

	

	def reset(self):
		global start_time
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

	def on_enter_normal(self):
		pass

	def execute_normal(self):
		pass	

	def on_exit_normal(self):
		pass
	def getPointToGo1(self,point, theta):
		x = point.x - (1.5 * BOT_RADIUS) * (math.cos(theta))
		y = point.y - (1.5 * BOT_RADIUS) * (math.sin(theta))
		return Vector2D(int(x), int(y))

	def on_enter_setStance(self):
		print("entered setstance hello")
		self.theta = normalize_angle(angle_diff( self.kub.state.ballPos,self.target_point))
		self.go_at = self.getPointToGo1(self.kub.state.ballPos, self.theta)
		_GoToPoint_.init(self.kub, self.go_at, self.theta)
		pass


	def execute_setStance(self):
		global DIST_THRESH
		print("executing setstance fnfnfj")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
		print "-----------------------##########-"
		generatingfunction = _GoToPoint_.execute(start_time,DIST_THRESH,True)
		
		for gf in generatingfunction:
			self.kub,target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			theta = angle_diff(self.target_point, self.kub.state.ballPos)
			#print theta,"------------------------"
			self.go_at = self.getPointToGo1(self.kub.state.ballPos, theta)
			# #print vicinity_points(self.go_at,target_point,thresh=BOT_BALL_THRESH)
			# if(dist(self.go_at,self.kub.get_pos())<BOT_BALL_THRESH):
			# 	self.behavior_failed = True
			# # 	break
			# if not vicinity_points(self.go_at, target_point,thresh=BOT_RADIUS*2.0):
			# 	print "Finally"
			# 	self.behavior_failed = True 
			# 	break
			#print self.go_at.x,"1",self.go_at.y,"abc",self.kub.get_pos().x,"2",self.kub.get_pos().y
		print("main nikal gya")
		self.goto_kick = True;
			

	def on_enter_GoAndKick(self):
		print("entered GoAndKick")
		theta = self.kub.get_pos().theta
		_GoToPoint_.init(self.kub, self.kub.state.ballPos, theta)
		pass

	def execute_GoAndKick(self):
		print("ha ma pahuch gaya")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)  
		self.kub.kick(self.power)
		self.kub.execute()
		generatingfunction = _GoToPoint_.execute(start_time,BOT_BALL_THRESH)
		for gf in generatingfunction:
			self.kub,ballPos = gf
			
			if not vicinity_points(ballPos,self.kub.state.ballPos,thresh=1.5*BOT_RADIUS):
				self.behavior_failed = True
				break

	def on_exit_GoAndKick(self):
		print("kicking bolo all is well")
		self.kub.reset()
		self.kub.kick(self.power)
		self.kub.execute()
		self.goto_kick=False
		pass

	

	def on_exit_setStance(self):
		print("main firse nikl gya")
		self.goto_kick = True;
		pass

	def on_exit_turnAround(self):
		print("not exiting")
		pass


