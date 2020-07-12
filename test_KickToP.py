
print "In test GoToPoint"
from kubs import kubs, cmd_node
from velocity.run import *
import rospy,sys,os
import math
import time
from role import GoToBall, GoToPoint, KickToPointP
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import point_SF
from utils.math_functions import *
from utils.config import *
from krssg_ssl_msgs.srv import bsServer
import subprocess

BOT_ID = int(sys.argv[1])
x=int(sys.argv[2])
y=int(sys.argv[3])

target=Vector2D(x,y)

print "BOT_ID Received",BOT_ID
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)

GOAL_POINT = point_2d()
GOAL_POINT.x = 1000
GOAL_POINT.y = 1200
READY_TO_KICK = False
REPLANNED = 0
homePos = None
awayPos = None
state = None


rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
try:
	state = getState(state)
except rospy.ServiceException, e:
	print e
if state:
	BState = state.stateB

kub = kubs.kubs(BOT_ID, BState, pub)


st = None 
tnow = None 
planned = False
ramp_upt = 5.0
ramp_dnt = 0.0
ramp_rampt = 0.0
case = -1

mvw=1.0
mvx=1.0
mvy=1.0


def reset():
	global start_time
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

def GUI_Callback(data):
	global BOT_ID, kub, BState, pub
	BOT_ID = data.bot_id
	print BOT_ID, "_____________________________"
	kub = kubs.kubs(BOT_ID, BState, pub)




def function(id_,state):
	kub = kubs.kubs(id_,state,pub)
	kub.update_state(state)
	print(kub.kubs_id) 
	g_fsm = KickToPointP.KickToPoint(target)
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	g_fsm.add_theta(theta=normalize_angle(atan2(target.y - state.ballPos.y,target.x - state.ballPos.x)))
	g_fsm.get_pos_as_vec2d(target)
    #g_fsm.as_graphviz()
	#g_fsm.write_diagram_png()
	print('something before spin')
	g_fsm.spin()
	# 




def kp_callback(data):
	global st, planned, ramp_rampt, ramp_dnt, ramp_upt, case, tnow
	
	ballpos = data.ballPos
	theta = angle_diff(target, ballpos)
	go_at = getPointBehindTheBall(ballpos, theta, 2)

	print(radian_2_deg(theta))
	print(go_at.x)
	print(go_at.y)
	
	t = rospy.Time.now()
	t = t.secs + 1.0*t.nsecs/pow(10,9)
	print(" t - start = ",t-start_time)
	[vx, vy, vw, REPLANNED] = Get_Vel(start_time, t, BOT_ID, go_at, data.homePos, data.awayPos, True, 1.2)
		#vx, vy, vw, replanned
	print("-------------------  REPLANNED = ",REPLANNED)
	if(REPLANNED):
    		reset()
	
	print("vx = ",vx)
	print("vy = ",vy)
	
	mvx=vx
	mvy=vy

	# print("kubs_id = ",kub.kubs_id)

	curPos = Vector2D(int(data.homePos[BOT_ID].x),int(data.homePos[BOT_ID].y))
	#if vicinity_points(go_at, curPos, 4) == False:
	try:	
		kub.move(vx, vy)
		print(vw)
		print("#Test Statement ---- homePos")
		kub.turn(vw)
		kub.execute()
	except Exception as e:
		print("In except",e)
		pass
	print(dist(go_at, curPos))
	
	if vx == 0 and vy == 0:
    	

		print(data.homePos[BOT_ID].theta)
		print(theta)
		totalAngle = theta
		MAX_w = (MAX_BOT_OMEGA+MIN_BOT_OMEGA)/1.2
		# theta_left = float(homePos[kub_id].theta-totalAngle)
		theta_lft = normalize_angle(normalize_angle(data.homePos[BOT_ID].theta)-totalAngle)*-1.0+3.1412
		
		if abs(theta_lft)<ROTATION_FACTOR/100:
			vw = 0.0
			READY_TO_KICK=True
		else:
			READY_TO_KICK=False
			vw = (theta_lft/2*math.pi)*MAX_w

		if abs(vw)<1*MIN_BOT_OMEGA and READY_TO_KICK==False:
			vw = 1*MIN_BOT_OMEGA*(1 if vw>0 else -1)


		if vw!=0:
			print("TURNING")
		else:
			print("DONE")
			
		mvw=vw

		

		print "Omega Return",vw
		print "TRY: ",mvw
		print READY_TO_KICK
		print "___________________________________________________________________"
		kub.reset()
		kub.turn(vw)
		kub.execute()

		if READY_TO_KICK  :
    			print "\nAngle Remaining : ",theta_lft
    			print "      ____________REACHED BALL, READY TO KICK____________\n"
    			rospy.signal_shutdown("=======ALIGNED======")



if __name__ == "__main__":
	global start_time, st, planned, case, mvw
	case = -1
	st = None
	planned = False
	print("here")

	#Begin the process: Driver, START MOVING

	rospy.init_node('node_new',anonymous=False)
	start_time = rospy.Time.now()
	start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)


	pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
	print('testing...')
	sub1 = rospy.Subscriber('/belief_state', BeliefState, kp_callback, queue_size=1000)
	
	sub2 = rospy.Subscriber('/gui_params', point_SF, GUI_Callback, queue_size = 1000)

	print("Ho gya!!")
	rospy.spin()
	
	sub1.unregister()
	sub2.unregister()
	print "AB MAIN KARUNGAAA -------- KICKKK !!!"

command="python KickP.py %s %s %s"%(BOT_ID,target.x,target.y)
print(command)
os.system(command)
	


#cmd = "date"

#returns output as byte string
#returned_output = subprocess.check_output(cmd)

#using decode() function to convert byte string to string
#print('Current date is:', returned_output.decode("utf-8"))
