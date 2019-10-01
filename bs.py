import rospy
from krssg_ssl_msgs.msg import BeliefState
from utils.config import BS_ADDRESS
from krssg_ssl_msgs.srv import *
State = None

def BS_callback(state):
	global State 
	State = state
	print state

def bs(req):
	global State
	print State
	req.stateA = State
	return bsServerResponse(req.stateA)

def bsserver():
	rospy.init_node('BSnode',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
  	s = rospy.Service('bsServer',bsServer,bs)
	rospy.spin()
if __name__ == "__main__":
	bsserver()	

