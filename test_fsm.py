from role import _GoToPoint
from kubs import kubs, cmd_node
from velocity.run import *
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import point_SF
from utils.config import *
from utils.geometry import *
import sys

BOT_ID = int(sys.argv[1])
print "bot_id received",BOT_ID
pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)

state = None
kub = None

points = [
Vector2D(-2400.79174805,-1560.16650391),Vector2D(-2400.79174805,500.04296875) ,Vector2D(-300.57901001,500.04296875),
Vector2D(-300.57901001,-1560.16650391)

]
curr_pt = 0
def BS_callback(BState):
    global pub,BOT_ID,state,kub,start_time, curr_pt
    global points

    if not state:
        print("__________________________________")
        state = BState
        kub = kubs.kubs(BOT_ID, BState, pub)
        _GoToPoint.init(kub, points[0],0)
        print(kub.kubs_id)
        print "gfsdfj,fhjgcvkubvgfkhjub_______________________________"

    print("distance s",points[curr_pt].dist(Vector2D(BState.homePos[0].x,BState.homePos[0].y)))
    if points[curr_pt].dist(Vector2D(BState.homePos[0].x,BState.homePos[0].y)) < 300:
        curr_pt = (curr_pt+1)%4
        print("updates index",curr_pt)
        _GoToPoint.init(kub, points[curr_pt],0)
    kub.update_state(BState)
    print ("point index",curr_pt)
    _GoToPoint.execute(start_time,200,BState)
 
        


if __name__ == "__main__":
    global start_time
    rospy.init_node('node_new',anonymous=False)
    start_time = rospy.Time.now()
    start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
    # BS_callback(start_time)
    rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
    rospy.spin()

    