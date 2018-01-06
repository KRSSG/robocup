from tests import pid_tune
import rospy

rospy.init_node('node_new',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
pid_tune.main()