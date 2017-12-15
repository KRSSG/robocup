import rospy,sys
from utils.geometry import Vector2D
from utils.math_functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint
from multiprocessing import Process
from kubs import kubs
from math import atan2,pi
from utils.math_functions import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)




def g(id_,state):
	kub = kubs.kubs(id_,state,pub)
	print(kub.kubs_id)
	g_fsm = GoToPoint.GoToPoint(kub,kub.state.ballPos,normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.y)))
	# g_fsm = GoToBall.GoToBall(kub,normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.y)))
	# g_fsm.as_graphviz()
	# g_fsm.write_diagram_png()
	g_fsm.spin()
	# print
	# print kub.state.homePos[kub.kubs_id].theta,t

def BS_callback(state):
	g(0,state)

# for i in xrange(0):
# 	p = Process(target=g,args=(i,))
# 	p.start()

#g_fsm1 = GoToBall.GoToBall(kub,deg_2_radian(45))




#print str(kub.kubs_id) + str('***********')
rospy.init_node('node_new',anonymous=False)
start_time = rospy.Time.now()

start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
rospy.spin()

# kub1 = kubs.kubs(0,pub)
# # print kub1.state.ballPos.y,kub1.state.ballPos.x
# # while True:
# # 	pass
# g1_fsm = GoToBall.GoToBall(kub1,Vector2D(0,3000))
# g1_fsm.as_graphviz()
# g1_fsm.write_diagram_png()

# # g_fsm1.spin()
# g1_fsm.spin()
# #t = threading.Thread(target=g_fsm.spin())
# #t1 = threading.Thread(target=g1_fsm.spin())


# #t.start()
# #t1.start()*
