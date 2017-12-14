import rospy,sys
from utils.geometry import Vector2D
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import GoToPoint
from kubs import kubs
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)


kub = kubs.kubs(0,pub)
g_fsm = GoToPoint.GoToPoint(kub,Vector2D(-4000,27))
g_fsm.as_graphviz()
g_fsm.write_diagram_png()
g_fsm.spin()