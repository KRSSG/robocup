import rospy,sys
from utils.geometry import Vector2D
from utils.math_functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from multiprocessing import Process
from kubs import kubs
from math import atan2,pi
from utils.math_functions import *

from tactics import  sample_tactic

pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=False)

# flag = True
def function(id_,state):
	global flag
	kub = kubs.kubs(id_,state,pub)
	# print(kub.kubs_id)
	g_fsm = sample_tactic.SampleTactic()
	# print(kub.kubs_id+1)
	g_fsm.add_kub(kub)
	# print(kub.kubs_id+2)

	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	g_fsm.spin_cb()
	# print(kub.kubs_id+3)


def BS_callback(state):

	function(0,state)




rospy.init_node('node_new',anonymous=False)
start_time = rospy.Time.now()

start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   



while True:
	state = shared.get('state')
	if state:
		function(0,state)
		break

