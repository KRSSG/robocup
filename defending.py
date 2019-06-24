import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint, top , bottom
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


def function1(id_,state,pub):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = top.defender()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	g_fsm.add_theta(theta=math.pi)
	print('something before spin')
	g_fsm.spin()

def function2(id_,state,pub):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = bottom.defender()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	g_fsm.add_theta(theta=math.pi)
	print('something before spin')
	g_fsm.spin()

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
	            print("process 1")
	            function1(1,state.stateB,pub)
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
	            print("process 2")
	            function2(2,state.stateB,pub)
	            #p2.start()
	            #p1.join()
	            #p2.join()
	            # print('chal ja')
	            # break
	    #rospy.spin()	


#print str(kub.kubs_id) + str('***********')
p1 = multiprocessing.Process(target=main1, args=(0,))
p2 = multiprocessing.Process(target=main2, args=(1,))
p1.start()
p2.start()
p1.join()
p2.join()
#main1()

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)




