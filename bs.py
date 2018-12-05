import rospy
from krssg_ssl_msgs.msg import BeliefState
import memcache
from utils.config import BS_ADDRESS
shared = memcache.Client(BS_ADDRESS,debug=False)


def BS_callback(state):
	shared.set('state',state)
	print state

rospy.init_node('bs_memcache_node',anonymous=False)
rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
rospy.spin()
