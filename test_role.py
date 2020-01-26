import rospy
from krssg_ssl_msgs.msg import gr_Commands
from utils.geometry import Vector2D
from utils.functions import *
from role import  GoToBall, GoToPoint
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi

rospy.init_node('node',anonymous=False)
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

def function(id_,state):	
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	# g_fsm = GoToBall.GoToBall()
	g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	g_fsm.add_point(point=Vector2D(1000,1000),orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	# g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	g_fsm.spin()
	
while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("Error ",e)	
	if state:
		function(1,state.stateB)
		print('chal ja')
		break
# rospy.spin()	
