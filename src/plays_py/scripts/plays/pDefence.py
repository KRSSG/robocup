import sys

sys.path.append('../utils/')
sys.path.append('../')
sys.path.insert(0,'../../')
sys.path.insert(0,'../../../tactics_py/scripts/')

from utils import tactics_union
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from Defence_node_function import assign_roles
import rospy
import threading
import tactic_factory

class pDefence(object):
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

        self.role_list[0][0] = "TMark"
        self.role_list[0][1] = params

        self.role_list[1][0] = "TPressureCooker"
        self.role_list[1][1] = params

        self.role_list[2][0] = "TWall"
        self.role_list[2][1] = params


        self.role_list[4][0] = "TGoalie"
        self.role_list[4][1] = params

    def tactic_instance(self, bot_id, tactic_id, params, other_bot_id=-1, wall=[None, None]):
        
        
        if tactic_id == "TMark":        
            print("executing mark")
            # instance = tactic_factory.TStop.TStop(bot_id, self.state, params)
            instance = tactic_factory.TMark.TMark(bot_id, self.state,  params)
        elif tactic_id == "TPressureCooker":
            instance = tactic_factory.TPressureCooker.TPressureCooker(bot_id, self.state, params)
        elif tactic_id == "TWall":
            print("botid for wall",bot_id)
            print("WAAAAALL",wall)
            instance = tactic_factory.TWall.TWall(bot_id, self.state, params, wall)
        elif tactic_id == "TPrimaryDefender":
            #print("rahul cjhutiya hai")
            instance  = tactic_factory.TPrimaryDefender.TPrimaryDefender(bot_id, self.state, params)
        elif tactic_id == "TGoalie":
            #print "&&&&&&&&&&&&&&&&GOALIE&&&&&&&&&&&&&&&&&&&&&"
            instance  = tactic_factory.TGoalie.TGoalie(bot_id, self.state, params)
        ##################################################################################
        elif tactic_id == "TAttackerOpp":
            instance  = tactic_factory.TAttacker.TAttacker(bot_id, self.state, params)
        ##################################################################################
        else:
            print("mark is last option")
            instance = tactic_factory.TMark.TMark(bot_id, self.state, bot_to_mark, params)
        return instance


    def bs_callback(self, data):
        """
        update the belief belief_state
        """
        #print "in callback function here"
        self.state.isteamyellow                 = not data.isteamyellow
        self.state.frame_number                 = data.frame_number
        self.state.t_capture                    = data.t_capture
        self.state.ballPos                      = data.ballPos
        self.state.ballVel                      = data.ballVel
        self.state.homePos                      = data.awayPos
        self.state.awayPos                      = data.homePos
        self.state.homeVel                      = data.awayVel
        self.state.awayVel                      = data.homeVel
        self.state.ballDetected                 = data.ballDetected
        self.state.awayDetected                 = data.homeDetected
        self.state.homeDetected                 = data.awayDetected
        self.state.opp_bot_closest_to_ball      = data.our_bot_closest_to_ball
        self.state.our_bot_closest_to_ball      = data.opp_bot_closest_to_ball
        self.state.opp_goalie                   = data.our_goalie
        self.state.our_goalie                   = data.opp_goalie
        self.state.opp_bot_marking_our_attacker = data.opp_bot_marking_our_attacker
        self.state.ball_at_corners              = data.ball_at_corners
        self.state.ball_in_our_half             = data.ball_in_our_half
        self.state.ball_in_our_possession       = data.ball_in_our_possession

        # self.print_belief_state()
        self.detected = 0
        for idx in range(len(self.state.homeDetected)):
            if self.state.homeDetected[idx]==True:
                self.detected += 1

        ##self.print_belief_state()
        global assigned_role
        assigned_role=assign_roles(self.state)    #ATTACKER, PD, MD, MD, FORWARD
        
        #print("_____________Ass______",assigned_role)
        tac_1 = self.tactic_instance(assigned_role[0], self.role_list[4][0], self.role_list[4][1])     #goalie
        tac_1.updateParams(self.state)
        tac_1.execute(self.state, self.pub)

        tac_2 = self.tactic_instance(assigned_role[1][0], self.role_list[1][0], self.role_list[1][1])     #pressurecooker
        tac_2.updateParams(self.state)
        tac_2.execute(self.state, self.pub, assigned_role[1][1])

        tac_3 = self.tactic_instance(assigned_role[2][0], self.role_list[0][0], self.role_list[0][1],assigned_role[2][1])        #Mark
        tac_3.updateParams(self.state)
        tac_3.execute(self.state, self.pub, assigned_role[2][1])

        tac_4 = self.tactic_instance(assigned_role[3][0], self.role_list[0][0], self.role_list[0][1], assigned_role[3][1])        #Mark
        tac_4.updateParams(self.state)
        tac_4.execute(self.state, self.pub, assigned_role[3][1])


        wall_defender = [assigned_role[4], assigned_role[5]]
        print("chutiya deff ", wall_defender)
        tac_5 = self.tactic_instance(assigned_role[4], self.role_list[2][0], self.role_list[2][1],1, wall_defender)        #PrimaryDefender
        tac_5.updateParams(self.state)
        tac_5.execute(self.state, self.pub)
        # tac_5.execute(self.state,self.pub)

        tac_6 = self.tactic_instance(assigned_role[5], self.role_list[2][0], self.role_list[2][1],1, wall_defender)        #PrimaryDefender
        tac_6.updateParams(self.state)
        tac_6.execute(self.state, self.pub)

    def execute(self):
        print "Initializing the node "
        rospy.init_node('play_py_node',anonymous=False)
        self.pub = rospy.Publisher('/grsim_data', gr_Commands, queue_size=1000)
        rospy.Subscriber('/belief_state', BeliefState, self.bs_callback, queue_size=1000)
        rospy.spin()            

                    




    

