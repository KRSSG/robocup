KUB_ID = 1

import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *

from tactics import  Goalie

pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

# _Goalie_.main()
flag = True
def function(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	# global flag
	# print(kub.kubs_id)
	g_fsm = Goalie.Goalie()
	# print(kub.kubs_id+1)
	g_fsm.add_kub(kub)
	# print(kub.kubs_id+2)

	# g_fsm.as_graphviz()
	# g_fsm.write_diagram_png()
	g_fsm.spin()
	# print(kub.kubs_id+3)


rospy.init_node('goalie',anonymous=False)
start_time = rospy.Time.now()

start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   



while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print e
	if state:
		function(KUB_ID,state.stateB)
		# break
rospy.spin()
		

