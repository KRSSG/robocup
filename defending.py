import rospy,sys
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from role import defenders
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
import multiprocessing
import threading

def1 = int(sys.argv[1])
def2 = int(sys.argv[2])

def function1(id_1,id_2,state,pub):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = defenders.defender(id_1,id_2,"top")
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function2(id_1,id_2,state,pub):
	kub = kubs.kubs(id_2,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = defenders.defender(id_2,id_1,"bottom")
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def main1(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("service_exception")		
	    if state:
	            print("process 1")
	            function1(def1,def2,state.stateB,pub)

def main2(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("service_exception")		
	    if state:
	            print("process 2")
	            function2(def1,def2,state.stateB,pub)

p1 = multiprocessing.Process(target=main1, args=(0,))
p2 = multiprocessing.Process(target=main2, args=(1,))
p1.start()
p2.start()
p1.join()
p2.join()





