import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *



import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=True)



def function(id_,state,pub):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	# g_fsm = GoToBall.GoToBall()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm = KickToPoint.KickToPoint(Vector2D(-HALF_FIELD_MAXX, 0))
	g_fsm.add_kub(kub)
	theta = angle_diff(state.ballPos, Vector2D(-HALF_FIELD_MAXX, 0))
	g_fsm.add_theta(theta)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	# g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	#print('something before spin')
	g_fsm.spin()
	# 



pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
rospy.init_node('node',anonymous=False)

while True:
	state = None
	#state=shared.get('state')
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("chutiya")		
	if state:
		#print('lasknfcjscnajnstate',state.stateB.homePos)
		function(1,state.stateB,pub)
		#print('chal ja')
		# break
rospy.spin()	




