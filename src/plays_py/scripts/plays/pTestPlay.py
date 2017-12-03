import sys

sys.path.append('../utils/')
sys.path.append('../')
sys.path.insert(0,'../../')
sys.path.insert(0,'../../../tactics_py/scripts/')

from utils import tactics_union
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
import rospy
import threading
import tactic_factory

class pTestPlay(object):
    def __init__(self):
        self.state = BeliefState() 
        self.detected = 6
        
        self.Tactic_0 = False
        self.Tactic_1 = False
        self.Tactic_2 = False
        self.Tactic_3 = False
        self.Tactic_4 = False
        self.Tactic_5 = False
        self.flag=[1]
        self.role_list = [['' for i in range(2)] for j in range(self.detected+1)]

        params = tactics_union.Param()
        params.IntercptP.awayBotID = 3
        params.IntercptP.where = 0
        
        self.role_list[0][0] = "TTestIt"
        self.role_list[0][1] = params

    def tactic_instance(self, bot_id, tactic_id, params, other_bot_id=-1):
        
        
        if tactic_id == "TTestIt":
            instance = tactic_factory.TTestIt.TTestIt(bot_id, self.state, params)
        else:
            instance=None
        return instance

    def bs_callback(self, data):
        self.state.isteamyellow                 = data.isteamyellow
        self.state.frame_number                 = data.frame_number
        self.state.t_capture                    = data.t_capture
        self.state.ballPos                      = data.ballPos
        self.state.ballVel                      = data.ballVel
        self.state.awayPos                      = data.awayPos
        self.state.homePos                      = data.homePos
        self.state.awayVel                      = data.awayVel
        self.state.homeVel                      = data.homeVel
        self.state.ballDetected                 = data.ballDetected
        self.state.homeDetected                 = data.homeDetected
        self.state.awayDetected                 = data.awayDetected
        self.state.our_bot_closest_to_ball      = data.our_bot_closest_to_ball
        self.state.opp_bot_closest_to_ball      = data.opp_bot_closest_to_ball
        self.state.our_goalie                   = data.our_goalie
        self.state.opp_goalie                   = data.opp_goalie
        self.state.opp_bot_marking_our_attacker = data.opp_bot_marking_our_attacker
        self.state.ball_at_corners              = data.ball_at_corners
        self.state.ball_in_our_half             = data.ball_in_our_half
        self.state.ball_in_our_possession       = data.ball_in_our_possession

        self.detected = 0
        for idx in range(len(self.state.homeDetected)):
            if self.state.homeDetected[idx]==True:
                self.detected += 1

       
        tac_1 = self.tactic_instance(1, self.role_list[0][0], self.role_list[0][1])     #attacker
        tac_1.updateParams(self.state)
        tac_1.execute(self.state, self.pub, self.flag)

        

    def execute(self):
        print "Initializing the node "
        rospy.init_node('play_py_node',anonymous=False)
        self.pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
        rospy.Subscriber('/belief_state', BeliefState, self.bs_callback, queue_size=1000)
        rospy.spin()            

                    




    

