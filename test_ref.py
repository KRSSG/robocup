import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  Referee, GoToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from krssg_ssl_msgs.srv import ref_comm
from krssg_ssl_msgs.msg import Ref
from math import atan2,pi
from utils.functions import *
import time
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

# def ref_callback(data):
	# rospy.loginfo(data.command)


# rospy.Subscriber('/ref_data',Ref,ref_callback)
def function(id_,state,ref_state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	print("11")
	# print(ref_state)
	g_fsm = Referee.Referee(ref_state)
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
	print("enter")
	state = None
	ref_state = None
	#state=shared.get('state')
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	# print("11")
	rospy.wait_for_service('ref_comm',)
	getrState = rospy.ServiceProxy('ref_comm',ref_comm)
	# print("12")
	try:
		state = getState(state)
		ref_state = getrState(ref_state)
	except rospy.ServiceException, e:
		print("chutiya")		
	# print(state)
	# print(ref_state)
	if state and ref_state:
		print('lasknfcjscnajnstate',state.stateB.homePos)
		# print(ref_state.stateB)
		function(1,state.stateB,ref_state.stateB)
		print('chal ja')
		time.sleep(5)
rospy.spin()	




