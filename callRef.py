import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import Ref
from role import _GoToPoint_, GoToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import *
from math import atan2,pi
from utils.functions import *
import multiprocessing
import threading
import time
#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
command = 5
flag = 0
pcommand = 12
rospy.wait_for_service('ref_comm',)
getrState = rospy.ServiceProxy('ref_comm',ref_comm)

def kickoff_Blue(id_1,state,pub, target):
	kub = kubs.kubs(id_1,state,pub)
	kub.update_state(state)
	print("HEREEE")
	# time.sleep(2)
	gotopoint_fsm = GoToPoint.GoToPoint()
	gotopoint_fsm.add_kub(kub)
	gotopoint_fsm.add_point(target, 0)
	gotopoint_fsm.spin()
	kub.reset()
	kub.execute()
	# _GoToPoint_.init(kub,target,0)
	# start_time = rospy.Time.now()
	# start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
	# generatingfunction = _GoToPoint_.execute(start_time,DISTANCE_THRESH/3,True)
	# #self.behavior_failed = False
	# for gf in generatingfunction:
	#     kub,target_point = gf
	#     # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
	#     if not vicinity_points(target,target_point,BOT_RADIUS*2.0):
	#         # self.behavior_failed = True
	#         break
	#     if vicinity_points(kub.get_pos(), target,BOT_RADIUS*1.5):
	#     	print("bot", id_1)
	#     	print("reached")	
	#     	kub.reset()
	#     	kub.execute()
	#     	time.sleep(2)    	
	#     	break

def main1(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute()
	            print("process 1",command)
	            if command == 5:
	            	target = Vector2D(-1500,0)
	            	kickoff_Blue(1,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()	

def main2(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass	

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute()
	            print("process 2 ",command)
	            if command == 5:
	            	target = Vector2D(-1500,-1120)
	            	kickoff_Blue(2,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()	
def main3(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute()
	            print("process 3 ",command)
	            if command == 5:
	            	target = Vector2D(-550,0)
	            	kickoff_Blue(3,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break	
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()
def main4(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute() 
	            print("process 4 ",command)
	            if command == 5:
	            	target = Vector2D(-2500,0)
	            	kickoff_Blue(4,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()
def main5(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute()
	            print("process 5 ",command)
	            if command == 5:
	            	target = Vector2D(-3600,0)
	            	kickoff_Blue(5,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()
def main6(process_id,pub):
	global command
	# pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	try :
		rospy.init_node('node' + str(process_id),anonymous=False)
	except:
		pass

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
	            kub = kubs.kubs(process_id,state,pub)
	            kub.reset()
	            kub.execute()
	            print("process 6 ",command)
	            if command == 5:
	            	target = Vector2D(-1500,1120)
	            	kickoff_Blue(0,state.stateB,pub,target)
	            	kub.reset()
	            	kub.execute()
	            	print(process_id)
	            	print("COMPLETED")
	            	time.sleep(1)
	            	break
	            else:
	            	break
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	# rospy.spin()
# def main(comm,pub):

# 	global command 
# 	command = comm
# 	# print(command)
# 	# time.sleep(2)
# 	# print(pub.get_num_connections())
# 	# print("##########################################################################")
# 	# time.sleep(3)
# 	p1 = multiprocessing.Process(target=main1, args=(1,pub,))
# 	p2 = multiprocessing.Process(target=main2, args=(2,pub,))
# 	p3 = multiprocessing.Process(target=main3, args=(3,pub,))
# 	p4 = multiprocessing.Process(target=main4, args=(4,pub,))
# 	p5 = multiprocessing.Process(target=main4, args=(5,pub,))
# 	p6 = multiprocessing.Process(target=main6, args=(0,pub,))		
# 	p1.start()
# 	p2.start()
# 	p3.start()
# 	p4.start()
# 	p5.start()
# 	p6.start()
# 	p1.join()
# 	p2.join()
# 	p3.join()
# 	p4.join()
# 	p5.join()
# 	p6.join()
# 	print("COMMAND COMPLETED")
# 	time.sleep(2)
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
while True:
	ref_state = None
	ref_state = getrState(ref_state).stateB
	print ref_state.command
	# time.sleep(2)
	if ref_state.command == 2 or ref_state.command == 3 or pcommand==ref_state.command:
		continue
	command = ref_state.command
	print("command: ",command)
	print("pcommand: ", pcommand)
	pcommand = command
	# time.sleep(2)
	p1 = multiprocessing.Process(target=main1, args=(1,pub))
	p2 = multiprocessing.Process(target=main2, args=(2,pub))
	p3 = multiprocessing.Process(target=main3, args=(3,pub))
	p4 = multiprocessing.Process(target=main4, args=(4,pub))
	p5 = multiprocessing.Process(target=main5, args=(5,pub))
	p6 = multiprocessing.Process(target=main6, args=(0,pub))		
	p1.start()
	p2.start()
	p3.start()
	p4.start()
	p5.start()
	p6.start()
	p1.join()
	p2.join()
	p3.join()
	p4.join()
	p5.join()
	p6.join()
	print("COMMAND COMPLETED")		
#main1()

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)