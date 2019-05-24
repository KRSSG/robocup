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
		turnAround = 3
		GoAndKick = 4

	def __init__(self,target,continuous=False):

		super(KickToPoint,self).__init__()

		self.name = "KickToPoint"

		self.power = 7.0

		self.target_point = target 

		self.go_at = None 

		self.setStanceThresh = 1.5*BOT_RADIUS

		self.ball_dist_thresh = BOT_BALL_THRESH

		self.behavior_failed = False

		self.add_state(KickToPoint.State.normal,
			behavior.Behavior.State.running)

		self.add_state(KickToPoint.State.setStance,
			behavior.Behavior.State.running)

		self.add_state(KickToPoint.State.turnAround,
			behavior.Behavior.State.running)
		
		self.add_state(KickToPoint.State.GoAndKick,
			behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
			KickToPoint.State.normal,lambda: True,'immediately')

		self.add_transition(KickToPoint.State.normal,
			KickToPoint.State.setStance,lambda: self.setstance(),'far_away')

		self.add_transition(KickToPoint.State.normal,
			KickToPoint.State.turnAround,lambda: self.turnAroundDirect(),'near_enough')

		self.add_transition(KickToPoint.State.normal,
			KickToPoint.State.GoAndKick,lambda:self.GoAndKickDirect(),'very_close')

		self.add_transition(KickToPoint.State.setStance,
			KickToPoint.State.turnAround,lambda:self.turnaround(),'turn')

		self.add_transition(KickToPoint.State.turnAround,
			KickToPoint.State.GoAndKick,lambda: self.goandkick(),'kick')

		self.add_transition(KickToPoint.State.GoAndKick,
			behavior.Behavior.State.completed,lambda: self.at_ball_pos(),'kicked!!!')

		self.add_transition(KickToPoint.State.setStance,
			behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

		self.add_transition(KickToPoint.State.turnAround,
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
		theta = angle_diff(self.target_point, self.kub.state.ballPos)
		go_at = getPointToGo(self.kub.state.ballPos, theta) 
		return go_at.dist(self.get_pos_as_vec2d(self.kub.get_pos())) >= DIST_THRESH*0.85

	def turnaround(self):
		return not self.bot_moving() #and not self.facing_the_target() and self.near_targetBall_line() and self.one_more()

	def turnAroundDirect(self):
		return not self.setstance() and not self.bot_moving()

	def GoAndKickDirect(self):
		return self.facing_the_target() and not self.setstance()

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
		if vicinity_theta( self.theta, normalize_angle(self.kub.get_theta()), thresh=0.30 ):
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

	def on_enter_setStance(self):
		print("entered setstance")
		theta = angle_diff(self.target_point, self.kub.state.ballPos)
		self.go_at = getPointToGo(self.kub.state.ballPos, theta)
		_GoToPoint_.init(self.kub, self.go_at, self.theta)
		pass

	def reset(self):
		global start_time
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

	def execute_normal(self):
		pass	

	# def execute_setStance(self):
	# 	global start_time
	# 	t = rospy.Time.now()
	# 	t = t.secs + 1.0*t.nsecs/pow(10,9)
	# 	#print(" t - start = ",t-start_time)
	# 	theta = angle_diff(self.target_point, self.kub.state.ballPos)
	# 	go_at = getPointToGo(self.kub.state.ballPos, theta)
	# 	#print("goo", go_at)
	# 	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, self.kub.kubs_id, Vector2D(go_at.x, go_at.y), self.kub.state.homePos, self.kub.state.awayPos, True)
	# 		#vx, vy, vw, replanned
	# 	#print("-------------------REPLANNED = ",REPLANNED)
	# 	if(REPLANNED):
	# 		self.reset()
		
	# 	# print("kubs_id = ",kub.kubs_id)
	# 	curPos = self.kub.get_pos()
	# 	#if vicinity_points(go_at, curPos, 4) == False:
	# 	try:   
	# 		#print("vx = ",vx)
	# 		#print("vy = ",vy)
	# 		self.kub.move(vx, vy)
	# 		#print(vw)
	# 		#print("homePos")
	# 		self.kub.turn(vw)
	# 		self.kub.execute()
	# 	except Exception as e:
	# 		print("In except",e)
	# 		pass
	# 	print("exiting")

	def execute_setStance(self):
		global DIST_THRESH
		print("executing setstance")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,DIST_THRESH,True)
		for gf in generatingfunction:
			self.kub,target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			theta = angle_diff(self.target_point, self.kub.state.ballPos)
			self.go_at = getPointToGo(self.kub.state.ballPos, theta)
			if not vicinity_points(self.go_at,target_point,thresh=BOT_BALL_THRESH):
				self.behavior_failed = True
				break


	def on_enter_turnAround(self):
		print("entered turnAround")
		print(self.get_pos_as_vec2d(self.kub.get_pos()).dist(self.get_pos_as_vec2d(self.kub.state.ballPos)))
		self.theta = normalize_angle(angle_diff(self.kub.state.ballPos,self.target_point))
		_turnAround_.init(self.kub, self.theta )
		pass

	# def execute_turnAround(self):
	# 	print("TURNING")
	# 	#print(data.homePos[BOT_ID].theta)
	# 	#print(theta)
	# 	theta = angle_diff(self.target_point, self.kub.state.ballpos)
	# 	totalAngle = theta
	# 	MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
	# 	# theta_left = float(homePos[kub_id].theta-totalAngle)
	# 	theta_lft = normalize_angle(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta)-totalAngle)*-1.0+3.1412
	# 	vw = (theta_lft/2*math.pi)*MAX_w
	# 	# print "totalAngle",radian_2_deg(totalAngle)
	# 	# print "theta_left ",radian_2_deg(theta_lft),theta_lft
	# 	# print "homePos theta ",radian_2_deg(normalize_angle(homePos[kub_id].theta))
	# 	# print "omega ",vw
	# 	if abs(vw)<1*MIN_BOT_OMEGA:
	# 		vw = 1*MIN_BOT_OMEGA*(1 if vw>0 else -1)

	# 	if abs(theta_lft)<ROTATION_FACTOR/2:
	# 		vw = 0.0

	# 	print "Omega return",vw
	# 	self.kub.reset()
	# 	self.kub.turn(vw)
	# 	self.kub.execute()

	def execute_turnAround(self):
		print("executing turnaround")
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _turnAround_.execute(start_time)
		for gf in generatingfunction:
			self.kub,final_theta = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			if not vicinity_theta(self.theta,final_theta,thresh=0.1):
				self.behavior_failed = True
				print("failed")
				break


	def on_enter_GoAndKick(self):
		print("entered GoAndKick")
		theta = self.kub.get_pos().theta
		_GoToPoint_.init(self.kub, self.kub.state.ballPos, theta)
		pass

	# def on_enter_GoAndKick(self):
	# 	print("entered GoAndKick")
	# 	global alpha, buffer_dist, start_time
	# 	start_time = rospy.Time.now()
	# 	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9) 
	# 	alpha = atan2(self.kub.state.ballPos.y - self.kub.get_pos().y,self.kub.state.ballPos.x - self.kub.get_pos().x ) - self.kub.state.homePos[self.kub.kubs_id].theta
	# 	buffer_dist = self.get_pos_as_vec2d(self.kub.get_pos()).dist(self.get_pos_as_vec2d(self.kub.state.ballPos))*abs(sin(alpha))
	# 	theta = self.kub.get_pos().theta
	# 	_GoToPoint_.init(self.kub, self.kub.state.ballPos, theta)
	# 	pass

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

	# def execute_GoAndKick(self):
	# 	print("ha ma pahuch gaya")
	# 	global alpha, buffer_dist, start_time
	# 	time = rospy.Time.now()
	# 	time = 1.0*time.secs + 1.0*time.nsecs/pow(10,9)
	# 	self.kub.kick(self.power)
	# 	self.kub.execute()	


		

	def on_exit_GoAndKick(self):
		print("kicking bolo all is well")
		#self.kub.kick(self.power)
		self.kub.reset()
		self.kub.kick(self.power)
		self.kub.execute()
		pass

	def on_enter_normal(self):
		pass

	def on_exit_normal(self):
		pass

	def on_exit_setStance(self):
		print("exiting setstance")
		pass

	def on_exit_turnAround(self):
		print("not exiting")
		pass


