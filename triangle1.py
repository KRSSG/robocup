print "In test GOToPoint"
from kubs import kubs, cmd_node
from velocity.run import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import point_SF
from krssg_ssl_msgs.msg import target_triangle
from utils.config import *
from krssg_ssl_msgs.srv import bsServer
import sys

BOT_ID = 0
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

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def target_Callback(msg):
	global GOAL_POINT
	GOAL_POINT.x = msg.tx0
	GOAL_POINT.y = msg.ty0

def BS_callback(data):
	global homePos, REPLANNED
	global awayPos, start_time, BState, kub, GOAL_POINT
	BState = data
	homePos = data.homePos
	awayPos = data.awayPos
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	# print(" t - start = ",t-start_time)
	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, GOAL_POINT, homePos, awayPos)	#vx, vy, vw, replanned
	# print("-------------------REPLANNED = ",REPLANNED)
	if(REPLANNED):
		reset()
	# print("vx = ",vx)
	# print("vy = ",vy)
	# print("kubs_id = ",kub.kubs_id)
	try:	
		kub.move(vx, vy)
		kub.turn(vw)
		kub.execute()
	except Exception as e:
		print("In except",e)
		pass	

if __name__ == "__main__":
	global start_time
	rospy.init_node('node_new_1',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)	
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	rospy.Subscriber('/target_params', target_triangle, target_Callback, queue_size = 1000)
	rospy.spin()