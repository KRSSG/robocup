import rospy
from krssg_ssl_msgs.msg import Ref
import memcache
# from utils.config import RC_ADDRESS
from krssg_ssl_msgs.srv import *
#shared = memcache.Client(BS_ADDRESS,debug=False)
State = None

def RC_callback(state):
	global State 
	State = state
	print("122")
	# print state

def rc(req):
	global State
	print State
	req.stateA = State
	print("12")
	return ref_commResponse(req.stateA)

def refcomm():
	rospy.init_node('RCnode',anonymous=False)
	rospy.Subscriber('/ref_data', Ref, RC_callback, queue_size=1000)
  	s = rospy.Service('ref_comm',ref_comm,rc)
  	print("11")
	rospy.spin()
if __name__ == "__main__":
	refcomm()	

