import sys

sys.path.append('../utils/')
sys.path.append('../')
sys.path.insert(0,'../../')
sys.path.insert(0,'../../../tactics_py/scripts/')

from utils import tactics_union
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from Stall_function import assign_roles
import rospy
import threading
import tactic_factory

class pStall(object):
    def __init__(self):
        self.state = BeliefState() 
        self.detected = 6
        
        self.Tactic_0 = False
        self.Tactic_1 = False
        self.Tactic_2 = False
        self.Tactic_3 = False
        self.Tactic_4 = False
        self.Tactic_5 = False

        self.role_list = [['' for i in range(2)] for j in range(self.detected+1)]

        params = tactics_union.Param()
        params.IntercptP.awayBotID = 3
        params.IntercptP.where = 0
        
        self.role_list[0][0] = "TAttacker"
        self.role_list[0][1] = params

        self.role_list[1][0] = "TPrimaryDefender"
        self.role_list[1][1] = params

        self.role_list[2][0] = "TMiddleDefender"
        self.role_list[2][1] = params

        self.role_list[3][0] = "TMiddleDefender"
        self.role_list[3][1] = params

        self.role_list[4][0] = "TForward"
        self.role_list[4][1] = params

        self.role_list[5][0] = "TGoalie"
        self.role_list[5][1] = params

        self.role_list[6][0] = "TReceiver"
        self.role_list[6][1] = params

    def tactic_instance(self, bot_id, tactic_id, params, other_bot_id=-1):
        
        
        if tactic_id == "TMiddleDefender":
            instance = tactic_factory.TMiddleDefender.TMiddleDefender(bot_id, self.state, other_bot_id, params)
        elif tactic_id == "TAttacker":
            instance = tactic_factory.TAttacker.TAttacker(bot_id, self.state, params)
        elif tactic_id == "TForward":
            instance = tactic_factory.TForward.TForward(bot_id, self.state, params)
        elif tactic_id == "TPrimaryDefender":
            instance  = tactic_factory.TPrimaryDefender.TPrimaryDefender(bot_id, self.state, params)
        elif tactic_id == "TGoalie":
            instance  = tactic_factory.TGoalie.TGoalie(bot_id, self.state, params)
        elif tactic_id == "TReceiver":
            instance  = tactic_factory.TReceiver.TReceiver(bot_id, self.state, params)
        else:
            instance = tactic_factory.TMark.TMark(bot_id, self.state, params)

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

        assigned_role=assign_roles(self.state)    #ATTACKER, PD, MD, MD, FORW, GOALIE   

        receiver_bot_id = [-1]
       
        tac_1 = self.tactic_instance(int(assigned_role[0]), self.role_list[0][0], self.role_list[0][1])     #attacker
        tac_1.updateParams(self.state)
        tac_1.execute(self.state, self.pub, receiver_bot_id)

        print(assigned_role, receiver_bot_id, "in pStall")

        tac_2 = self.tactic_instance(int(assigned_role[5]), self.role_list[5][0], self.role_list[5][1])     #Goalie
        tac_2.updateParams(self.state)
        tac_2.execute(self.state, self.pub)

        tac_3 = self.tactic_instance(int(receiver_bot_id[0]), self.role_list[6][0], self.role_list[6][1])   #Receiver
        tac_3.updateParams(self.state)
        tac_3.execute(self.state, self.pub, int(assigned_role[0]))

        tac=['','','','','']
        for i in [1,2,3,4]:

            if(receiver_bot_id[0]==assigned_role[i]):
                continue
            else:
                other_bot_id=-1
                if(i==2):                              #passing other_bot_id to middle_defender
                    other_bot_id=int(assigned_role[3])
                elif(i==3):
                    other_bot_id=int(assigned_role[2])    
                tac[i]=self.tactic_instance(int(assigned_role[i]), self.role_list[i][0], self.role_list[i][1], other_bot_id)
                tac[i].updateParams(self.state)
                tac[i].execute(self.state, self.pub)

    def execute(self):
        print "Initializing the node "
        rospy.init_node('play_py_node',anonymous=False)
        self.pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
        rospy.Subscriber('/belief_state', BeliefState, self.bs_callback, queue_size=1000)
        rospy.spin()            

                    




    

