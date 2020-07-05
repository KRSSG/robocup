import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint, pass_and_receive, receiver, passer
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
from tactics import Goalie
import multiprocessing
import threading
#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)


import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=True)


def function(id_1,id_2,state,pub):
	kub1 = kubs.kubs(id_1,state,pub)
	# print(id_1)
	kub1.update_state(state)
	kub2 = kubs.kubs(id_2,state,pub)
	# print(id_2)
	kub2.update_state(state)
	#print(kub1.kubs_id)
	p_fsm = passer.PassReceive()
	r_fsm = receiver.PassReceive()
	# g_fsm = GoToPoint.GoToPoint()
	if dist(kub1.state.ballPos,kub1.get_pos())< dist(kub2.state.ballPos,kub2.get_pos()):
		p_fsm.add_kub(kub1)
		p_fsm.add_kub(kub2)
		p_fsm.spin()
	else:
		r_fsm.add_kub(kub1)
		r_fsm.add_kub(kub2)
		r_fsm.spin()
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	#g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	# g_fsm.as_graphviz()
	# g_fsm.write_diagram_png()
	#print('something before spin')

def main1(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            # print("process 1")
	            function(1,2,state.stateB,pub)
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()	

def main2(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            # print("process 2")
	            function(2,1,state.stateB,pub)
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()	


#print str(kub.kubs_id) + str('***********')
p1 = multiprocessing.Process(target=main1, args=(1,))
p2 = multiprocessing.Process(target=main2, args=(2,))
p1.start()
p2.start()
p1.join()
p2.join()
#main1()

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)




