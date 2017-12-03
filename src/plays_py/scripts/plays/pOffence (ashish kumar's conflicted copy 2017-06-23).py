import sys

sys.path.append('../utils/')
sys.path.append('../')
sys.path.insert(0,'../../')
sys.path.insert(0,'../../../tactics_py/scripts/')

from utils import tactics_union
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from Offence_node_function import assign_roles_offence
import rospy
import threading
import tactic_factory

class pOffence(object):
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
        

        self.role_list[0][0] = "TAttackerX"
        self.role_list[0][1] = params


        self.role_list[3][0] = "TReceiver"
        self.role_list[3][1] = params


        self.role_list[1][0] = "TPrimaryDefender"
        self.role_list[1][1] = params


        self.role_list[2][0] = "TMidFielderX"
        self.role_list[2][1] = params


        self.role_list[4][0] = "TGoalie"
        self.role_list[4][1] = params

    def tactic_instance(self, bot_id, tactic_id, params, other_bot_id=-1):
        
        
        # if assigned_role==0:
        #     return tactic_factory.TStop.TStop(bot_id, self.state, params)

        if tactic_id == "TStop":
            # if(bot_id==assigned_role[2]):
            #   other_bot_id=assigned_role[3]
            # else:
            #   other_bot_id=assigned_role[2] 
              
            instance = tactic_factory.TStop.TStop(bot_id, self.state, params)
        elif tactic_id == "TAttackerX":
            instance = tactic_factory.TAttackerX.TAttackerX(bot_id, self.state, params)
        elif tactic_id == "TForward":
            instance = tactic_factory.TForward.TForward(bot_id, self.state, params)
        elif tactic_id == "TMidFielderX":
            instance = tactic_factory.TMidFielderX.TMidFielderX(bot_id, self.state, params)
        elif tactic_id =="TGoalie":
            instance = tactic_factory.TGoalie.TGoalie(bot_id, self.state, params) 
        elif tactic_id =="TPrimaryDefender":
            instance = tactic_factory.TPrimaryDefender.TPrimaryDefender(bot_id, self.state, params) 
        elif tactic_id =="TReceiver":
            instance = tactic_factory.TReceiver.TReceiver(bot_id, self.state, params) 
        else:
            instance = None

        return instance
    
    def bs_callback(self, data):
        """
        update the belief belief_state
        """
        #clearprint "in callback function here"
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

        # self.print_belief_state()
        print "in bs_cb"
        self.detected = 0
        for idx in range(len(self.state.homeDetected)):
            if self.state.homeDetected[idx]==True:
                self.detected += 1

        ##self.print_belief_state()
        assigned_role=assign_roles_offence(self.state)    #ATTACKER, PD, MD, MD, FORWARD
        
        receiver_bot_id = [-1]
        score_goal = [False]

        tac_1 = self.tactic_instance(int(assigned_role[1]), self.role_list[0][0], self.role_list[0][1])     #attacker
        tac_1.updateParams(self.state)
        tac_1.execute(self.state, self.pub, receiver_bot_id, score_goal)
        
        #receiver_bot_id[0] = -1
        
        if(receiver_bot_id[0] is not assigned_role[2]):
            tac_2 = self.tactic_instance(assigned_role[2], self.role_list[1][0], self.role_list[1][1])      #primary_defender
            tac_2.updateParams(self.state)
            tac_2.execute(self.state, self.pub)
        # else:
        #   tac_2 = self.tactic_instance(assigned_role[1], self.role_list[5][0], self.role_list[5][1])      #Receive
        #   tac_2.updateParams(self.state)
        #   tac_2.execute(self.state, self.pub)
        defenders = [assigned_role[3], assigned_role[4], assigned_role[5]]

        if(receiver_bot_id[0] is not assigned_role[3]):
            tac_3 = self.tactic_instance(assigned_role[3], self.role_list[2][0], self.role_list[2][1])      #middle_defender
            tac_3.updateParams(self.state)
            tac_3.execute(self.state, self.pub, assigned_role[1], defenders,score_goal[0])
        # else:
        #   tac_3 = self.tactic_instance(assigned_role[2], self.role_list[3][0], self.role_list[3][1])      #Receive
        #   tac_3.updateParams(self.state)
        #   tac_3.execute(self.state, self.pub, int(assigned_role[1]))

        if( receiver_bot_id[0] is not assigned_role[4]):
            tac_4 = self.tactic_instance(assigned_role[4], self.role_list[2][0], self.role_list[2][1])      #middle_defender
            tac_4.updateParams(self.state)
            tac_4.execute(self.state, self.pub, assigned_role[1], defenders, score_goal[0])
        else:
          tac_4 = self.tactic_instance(assigned_role[3], self.role_list[3][0], self.role_list[3][1])      #Receive
          tac_4.updateParams(self.state)
          tac_4.execute(self.state, self.pub, int(assigned_role[1]))

        if( receiver_bot_id[0] is not assigned_role[5]):
            tac_5 = self.tactic_instance(assigned_role[5], self.role_list[2][0], self.role_list[2][1])      #middle_defender
            tac_5.updateParams(self.state)
            tac_5.execute(self.state, self.pub, assigned_role[1], defenders, score_goal[0])
        else:
          tac_5 = self.tactic_instance(assigned_role[4], self.role_list[3][0], self.role_list[3][1])      #Receive
          tac_5.updateParams(self.state)
          tac_5.execute(self.state, self.pub, int(assigned_role[1]))


        tac_6 = self.tactic_instance(assigned_role[0], self.role_list[4][0], self.role_list[4][1])      #goalie
        tac_6.updateParams(self.state)
        tac_6.execute(self.state, self.pub)

    def execute(self):
        print "Initializing the node "
        rospy.init_node('play_py_node',anonymous=False)
        self.pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
        rospy.Subscriber('/belief_state', BeliefState, self.bs_callback, queue_size=1000)
        rospy.spin()            

                    




    

