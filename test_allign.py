import rospy,sys
from utils.geometry import *
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint , KickToPoint, allignTowardsPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)





def function(id_,state):
    kub = kubs.kubs(id_,state,pub)
    kub.update_state(state)
    print(kub.kubs_id)
    target = Vector2D(state.ballPos.x,state.ballPos.y)
    #ub.state.homePos[kub.kubs_id].theta += 3.1412
    g_fsm = allignTowardsPoint.allignTowardsPoint(target)
    # g_fsm = GoToPoint.GoToPoint()
    g_fsm.add_kub(kub)
    bot_ball = Vector2D(target.x - state.ballPos.x,target.y - state.ballPos.y)
    angle = bot_ball.tan_inverse()
    # g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
    g_fsm.add_theta(angle)
    #g_fsm.as_graphviz()
    #g_fsm.write_diagram_png()
    g_fsm.spin()
    print("kakaakka")
    # 



#print str(kub.kubs_id) + str('***********')
rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)

while True:
    state = None
    rospy.wait_for_service('bsServer',)
    getState = rospy.ServiceProxy('bsServer',bsServer)
    try:
            state = getState(state)
    except rospy.ServiceException, e:
            print("Error ",e)
    if state:
        function(1,state.stateB)
        # break
rospy.spin()




