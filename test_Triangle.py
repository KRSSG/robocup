import os
import rospy
from krssg_ssl_msgs.msg import target_triangle
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from utils.config import *
from math import sqrt
BOT_1 = 0
BOT_2 = 1
BOT_3 = 2

target_1 = point_2d()
target_2 = point_2d()
target_3 = point_2d()
homePos = None
awayPos = None
FIRST_CALL = 1
THRESHOLD = 2*BOT_BALL_THRESH
pub = rospy.Publisher('target_params', target_triangle)

def distance_(a, b):
    dx = a.x-b.x
    dy = a.y-b.y
    return sqrt(dx*dx+dy*dy)

def change_target():
	global target_1, target_2, target_3, homePos
	temp = point_2d()
	temp.x = target_1.x
	temp.y = target_1.y

	target_1.x = target_2.x
	target_1.y = target_2.y

	target_2.x = target_3.x
	target_2.y = target_3.y

	target_3.x = temp.x
	target_3.y = temp.y

def check_target():
	global target_1, target_2, target_3
	dist1 = distance_(target_1, homePos[BOT_1])
	dist2 = distance_(target_2, homePos[BOT_2])
	dist3 = distance_(target_3, homePos[BOT_3])

	flag1 = dist1<THRESHOLD
	flag2 = dist2<THRESHOLD
	flag3 = dist3<THRESHOLD

	print("flag1 = ",flag1," flag2 = ",flag2," flag3 = ",flag3)
	if(flag1 and flag2 and flag3):
		change_target()


def BS_callback(msg):
	global homePos, awayPos, FIRST_CALL, target_1, target_2, target_3
	homePos = msg.homePos
	awayPos = msg.awayPos
	msgs = target_triangle()
	msgs.tx0 = target_1.x
	msgs.ty0 = target_1.y

	msgs.tx1 = target_2.x
	msgs.ty1 = target_2.y

	msgs.tx2 = target_3.x
	msgs.ty2 = target_3.y

	check_target()
	if(FIRST_CALL):
		target_1 = homePos[BOT_2]
		target_2 = homePos[BOT_3]
		target_3 = homePos[BOT_1]
		FIRST_CALL = 0
	pub.publish(msgs)


if __name__ == "__main__":

	rospy.init_node('test_triangle',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	# change_target()
	rospy.spin()
