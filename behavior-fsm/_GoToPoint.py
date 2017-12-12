from kubs import kubs, cmd_node
from run import *
import rospy
sys.path.append('./../../plays_py/scripts/utils/')
from geometry import Vector2D
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from config import *

kub = None
start_time = None
GOAL_POINT = None

def init(_kub,target):
	global kub
	kub = _kub
	start_time = None
	GOAL_POINT = point_2d()
	GOAL_POINT.x = target.x
	GOAL_POINT.y = target.y

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def BS_callback(state):
	global GOAL_POINT, start_time
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	print(" t - start = ",t-start_time)
	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, GOAL_POINT, state.homePos, state.awayPos)	#vx, vy, vw, replanned
	print("-------------------REPLANNED = ",REPLANNED)
	if(REPLANNED):
		reset()
	print("vx = ",vx)
	print("vy = ",vy)	
	kub.move(vx, vy)
	kub.turn(vw)
	kub.execute(data)

def run():
	global start_time
	# rospy.init_node('node_new',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)	
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	rospy.spin()