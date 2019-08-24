print "In test GOToPoint"
from kubs import kubs, cmd_node
from velocity.run import *
import rospy,sys
import math
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import point_SF
from utils.math_functions import *
from utils.config import *
from krssg_ssl_msgs.srv import bsServer
import sys

BOT_ID = int(sys.argv[1])
print "bot_id received",BOT_ID
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)

GOAL_POINT = point_2d()
GOAL_POINT.x = 1000
GOAL_POINT.y = 1200
REPLANNED = 0
homePos = None
awayPos = None
state = None
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
try:
	state = getState(state)
except rospy.ServiceException, e:
	print e
if state:
	BState = state.stateB
kub = kubs.kubs(BOT_ID, BState, pub)
st = None 
tnow = None 
planned = False
ramp_upt = 5.0
ramp_dnt = 0.0
ramp_rampt = 0.0
case = -1
mvw = 0.0

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def GUI_Callback(data):
	global BOT_ID, kub, BState, pub
	BOT_ID = data.bot_id
	print BOT_ID, "_____________________________"
	kub = kubs.kubs(BOT_ID, BState, pub)

def kp_callback(data):
	global st, planned, ramp_rampt, ramp_dnt, ramp_upt, case, mvw, tnow
	target = Vector2D(0,0)
	ballpos = data.ballPos
	theta = angle_diff(target, ballpos)
	go_at = getPointToGo(ballpos, theta)
	print(radian_2_deg(theta))
	print(go_at.x)
	print(go_at.y)
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	print(" t - start = ",t-start_time)
	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, go_at, data.homePos, data.awayPos, True)
		#vx, vy, vw, replanned
	print("-------------------REPLANNED = ",REPLANNED)
	if(REPLANNED):
		reset()
	print("vx = ",vx)
	print("vy = ",vy)
	# print("kubs_id = ",kub.kubs_id)
	curPos = Vector2D(int(data.homePos[BOT_ID].x),int(data.homePos[BOT_ID].y))
	#if vicinity_points(go_at, curPos, 4) == False:
	try:	
		kub.move(vx, vy)
		print(vw)
		print("homePos")
		kub.turn(vw)
		kub.execute()
	except Exception as e:
		print("In except",e)
		pass
	print(dist(go_at, curPos))
	# if planned == False:
	# 	theta_lft = abs(normalize_angle(normalize_angle(data.homePos[BOT_ID].theta)-theta))
	# 	print(theta_lft)
	# 	if theta_lft >= 5.0*MAX_BOT_OMEGA:
	# 		case = 1
	# 		ramp_upt = 5.0
	# 		ramp_dnt = 5.0
	# 		ramp_rampt = (theta_lft-5.0*MAX_BOT_OMEGA)*1.0/MAX_BOT_OMEGA
	# 	elif theta_lft >= 2.5*MAX_BOT_OMEGA:
	# 		case = 2
	# 		ramp_upt = 0.0
	# 		ramp_dnt = 5.0
	# 		ramp_rampt = (theta_lft-2.5*MAX_BOT_OMEGA)*1.0/MAX_BOT_OMEGA
	# 	else:
	# 		case = 3
	# 		ramp_rampt = 0.0
	# 		ramp_dnt = sqrt(2.0*theta_lft/MAX_BOT_OMEGA_ACC)*1.0
	# 		mvw = sqrt(2.0*theta_lft*MAX_BOT_OMEGA_ACC)*1.0
	# 		ramp_upt = 0.0
	# 	planned = True
	# print(case)
	# print(ramp_rampt)
	# print(ramp_upt)
	# print(ramp_dnt)
	# print(mvw)
	if vx == 0 and vy == 0:
		print("TURNING")
		print(data.homePos[BOT_ID].theta)
		print(theta)
		totalAngle = theta
		MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
		# theta_left = float(homePos[kub_id].theta-totalAngle)
		theta_lft = normalize_angle(normalize_angle(data.homePos[BOT_ID].theta)-totalAngle)*-1.0+3.1412
		vw = (theta_lft/2*math.pi)*MAX_w
		# print "totalAngle",radian_2_deg(totalAngle)
		# print "theta_left ",radian_2_deg(theta_lft),theta_lft
		# print "homePos theta ",radian_2_deg(normalize_angle(homePos[kub_id].theta))
		# print "omega ",vw
		if abs(vw)<1*MIN_BOT_OMEGA:
			vw = 1*MIN_BOT_OMEGA*(1 if vw>0 else -1)

		if abs(theta_lft)<ROTATION_FACTOR/2:
			vw = 0.0

		print "Omega return",vw
		kub.reset()
		kub.turn(vw)
		kub.execute()
		# if st == None:
		# 	st = rospy.Time.now()
		# 	st = 1.0*st.secs + 1.0*st.nsecs/pow(10,9)
		# tnow = rospy.Time.now()
		# tnow = 1.0*tnow.secs + 1.0*tnow.nsecs/pow(10,9)
		# diff = tnow-st
		# if case == 1:
		# 	if diff <= 5.0:
		# 		vw = (tnow-st)*MAX_BOT_OMEGA_ACC
		# 	elif diff > 5.0 and diff <= 5.0+ramp_rampt:
		# 		vw = MAX_BOT_OMEGA
		# 	else:
		# 		tim = 10.0+ramp_rampt-diff
		# 		vw = tim*MAX_BOT_OMEGA_ACC
		# elif case == 2:
		# 	if diff <= ramp_rampt:
		# 		vw = MAX_BOT_OMEGA
		# 	else :
		# 		tim = 5.0+ramp_rampt-diff
		# 		vw = tim*MAX_BOT_OMEGA_ACC
		# else :
		# 	vw = mvw - MAX_BOT_OMEGA_ACC*diff 
		# if normalize_angle(homePos[BOT_ID].theta) < theta:
		# 	vw = -1.0*vw
		# print(vw)
		# kub.reset()
		# kub.turn(vw)
		# kub.execute()


if __name__ == "__main__":
	global start_time, st, planned, case, mvw
	case = -1
	mvw = 0.0
	st = None
	planned = False
	print("here")
	rospy.init_node('node_new',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
	print('testing...')
	rospy.Subscriber('/belief_state', BeliefState, kp_callback, queue_size=1000)
	rospy.Subscriber('/gui_params', point_SF, GUI_Callback, queue_size = 1000)
	rospy.spin()


