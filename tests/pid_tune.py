print("Pid tune imported")
import rospy,sys
from utils.geometry import Vector2D
from utils.math_functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
print("importing gotopoint")
from role import  GoToBall, GoToPoint
from multiprocessing import Process
from kubs import kubs
from math import atan2,pi
from utils.math_functions import *
from utils.config import *


import sys
# BOT_ID = sys.argv[1]
BOT_ID = 0
#ssl-vision-points
# points = [
# 	Vector2D(-2400.79174805,-1560.16650391),
# 	Vector2D(-2400.79174805,500.04296875) ,
# 	Vector2D(-300.57901001,500.04296875),
# 	Vector2D(-300.57901001,-1560.16650391)
# ]


def main():
	import memcache
	shared = memcache.Client(['127.0.0.1:11211'],debug=False)
	paddingX = 200	
	paddingY = 100

	points = [
		Vector2D((HALF_FIELD_MAXX-paddingX),(HALF_FIELD_MAXY-paddingY)),
		# Vector2D((HALF_FIELD_MAXX*0-paddingX),-(HALF_FIELD_MAXY-paddingY)),
		Vector2D((HALF_FIELD_MAXX-paddingX),-(HALF_FIELD_MAXY-paddingY)),
		# Vector2D(-(HALF_FIELD_MAXX-paddingX),(HALF_FIELD_MAXY-paddingY)),
	]
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	state = shared.get('state')
	kub = kubs.kubs(BOT_ID,state,pub)

	# while True:
	for p in points:
		print("Starting again")
		state = shared.get('state')
		kub.update_state(state)
		g_fsm = GoToPoint.GoToPoint()
		g_fsm.add_kub(kub)
		g_fsm.add_point(point=p,orient=state.homePos[kub.kubs_id].theta)
		g_fsm.spin()

if __name__ == '__main__':
	main()
