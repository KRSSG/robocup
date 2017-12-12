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

BOT_ID = 0
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
kub = kubs.kubs(BOT_ID, pub)
start_time = None
GOAL_POINT = point_2d()
GOAL_POINT.x = 1000
GOAL_POINT.y = 1200
REPLANNED = 0
homePos = None
awayPos = None
vx1 = None
vy1 = None

def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def BS_callback(data):
	global homePos, REPLANNED
	global awayPos, start_time
	homePos = data.homePos
	awayPos = data.awayPos
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	print(" t - start = ",t-start_time)
	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, data.ballPos, homePos, awayPos)	#vx, vy, vw, replanned
	print("-------------------REPLANNED = ",REPLANNED)
	print ("ball pos: ",data.ballPos.x, data.ballPos.y)
	print ("bot pos: ",data.homePos[BOT_ID].x, data.homePos[BOT_ID].y)
	if(REPLANNED ):
		reset()
		print("RETURNING-----------------")
		return
	print("vx = ",vx)
	print("vy = ",vy)	
	kub.move(vx, vy, vw)
	kub.execute(data)

if __name__ == "__main__":
	global start_time
	rospy.init_node('node_new',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)	
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	rospy.spin()