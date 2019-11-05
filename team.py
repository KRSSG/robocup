import rospy,sys
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from role import attackers,defenders,mid
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
import multiprocessing
import threading

def function1(id_1,id_2,state,pub):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = attackers.attacker(id_1,id_2)
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function2(id_1,id_2,state,pub):
	kub = kubs.kubs(id_2,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = attackers.attacker(id_2,id_1)
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function3(id_1,id_2,state,pub):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = defenders.defender(id_1,id_2,"top")
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function4(id_1,id_2,state,pub):
	kub = kubs.kubs(id_2,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = defenders.defender(id_2,id_1,"bottom")
	g_fsm.add_kub(kub)
	print('something before spin')
	g_fsm.spin()

def function5(id_,state,pub):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id)
	g_fsm = mid.mid(id_)
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
	            print("chutiya")		
	    if state:
	            print("process 1")
	            function1(3,1,state.stateB,pub)

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
	            print("chutiya")		
	    if state:
	            print("process 2")
	            function2(3,1,state.stateB,pub)

def main3(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            print("process 1")
	            function3(0,2,state.stateB,pub)

def main4(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            print("process 2")
	            function4(0,2,state.stateB,pub)

def main5(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            print("process 2")
	            function5(4,state.stateB,pub)

p1 = multiprocessing.Process(target=main1, args=(0,))
p2 = multiprocessing.Process(target=main2, args=(1,))
p3 = multiprocessing.Process(target=main3, args=(2,))
p4 = multiprocessing.Process(target=main4, args=(3,))
p5 = multiprocessing.Process(target=main5, args=(4,))

p1.start()
p2.start()
p3.start()
p4.start()
p5.start()
p1.join()
p2.join()
p3.join()
p4.join()
p5.join()

