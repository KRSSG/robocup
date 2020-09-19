import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint, Dribble_Kick
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

'''Arguments : (BOT_ID) (x coordinate of goal point) (y coordinate of goal point)'''
# Can try making it dynamic according to goalie's position, during gameplay...

BOT_ID = int(sys.argv[1])
x=int(sys.argv[2])
y=int(sys.argv[3])

target=Vector2D(x,y)

def function(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print("BOT ",kub.kubs_id ," IN ACTION ------")
	#ub.state.homePos[kub.kubs_id].theta += 3.1412
	g_fsm = Dribble_Kick.DribbleKick(target)
	g_fsm.add_kub(kub)
	#g_fsm.as_graphviz()
	#g_fsm.write_diagram_png()
	g_fsm.spin()


rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)

while True:
	state=None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print e	
	if state:
		function(BOT_ID,state.stateB)
		# break
rospy.spin()
