import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)





def function(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = GoToBall.GoToBall()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	print('something before spin')
	g_fsm.spin()
	# 



#print str(kub.kubs_id) + str('***********')
rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)

while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("Error ",e)	
	if state:
		print('lasknfcjscnajnstate',state.stateB.homePos)
		function(1,state.stateB)
		print('chal ja')
		# break
rospy.spin()	




