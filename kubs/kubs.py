import cmd_node
import rospy
from krssg_ssl_msgs.msg import BeliefState
from utils.math_functions import kub_has_ball

##
## @brief      Class for kubs.
##
class kubs:
    ##
    ## @brief      Constructs the object.
    ##
    ## @param      self     The object
    ## @param      kubs_id  The kubs identifier
    ##
    
    def __init__(self, kubs_id,state, pub):
        self.kubs_id = kubs_id
        self.pos = state.homePos[kubs_id]
        self.vx = 0
        self.vy = 0
        self.vw = 0
        self.isteamyellow = False
        self.dribbler = False
        self.power = False
        self.state = state
        # self.kubsBelief()
        self.pub = pub
        self.c=0
    ##
    ## @brief      { function_description }
    ##
    ## @param      self  The object
    ##
    ## @return     { description_of_the_return_value }
    ##
    
    def update_state(self,state):
        self.state = state
        self.pos = state.homePos[self.kubs_id]

    def reset(self):
        self.dribbler = False
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.power = 0.0
    
    ##
    ## @brief      { function_description }
    ##
    ## @param      self          The object
    ## @param      target_point  The target point
    ## @param      state         The state
    ##
    ## @return     { description_of_the_return_value }
    ##
    
    def move(self, vx, vy):
        self.vx = vx
        self.vy = vy

    ##
    ## @brief      { function_description }
    ##
    ## @param      self  The object
    ##
    ## @return     { description_of_the_return_value }
    ##
    

    def dribble(self, dribbler):
        self.dribbler = dribbler


    ##
    ## @brief      { function_description }
    ##
    ## @param      self   The object
    ## @param      angle  The angle
    ##
    ## @return     { description_of_the_return_value }
    ##

    def turn(self, vw):
        self.vw = vw

    ##
    ## @brief      { function_description }
    ##
    ## @param      self   The object
    ## @param      power  The power
    ##
    ## @return     { description_of_the_return_value }
    ##
    

    def kick(self, power):
        self.power = power



    ##
    ## @brief      { function_description }
    ##
    ## @param      self   The object
    ## @param      state  The state
    ##
    ## @return     { description_of_the_return_value }
    ##
    

    def execute(self):
        cmd_node.send_command(self.pub, self.isteamyellow, self.kubs_id, self.vx, self.vy, self.vw*0, self.power, self.dribbler)  
        self.reset()

    def has_ball(self):
        return kub_has_ball(self.state,self.kubs_id)

    ##
    ## @brief      Gets the position.
    ##
    ## @param      self   The object
    ## @param      state  The state
    ##
    ## @return     The position.
    ##
    
    def get_vel(self):
        velo = {
                'magnitute':magnitute(self.state.homeVel[self.kubs_id]),
                'direction':direction(self.state.homeVel[self.kubs_id])
                }
        return velo

    def get_pos(self):
        return self.state.homePos[self.kubs_id]

    # def bs_callback(self, data):
    #     self.state.isteamyellow                 = data.isteamyellow
    #     self.state.frame_number                 = data.frame_number
    #     self.state.t_capture                    = data.t_capture
    #     self.state.ballPos                      = data.ballPos
    #     self.state.ballVel                      = data.ballVel
    #     self.state.awayPos                      = data.awayPos
    #     self.state.homePos                      = data.homePos
    #     self.state.awayVel                      = data.awayVel
    #     self.state.homeVel                      = data.homeVel
    #     self.state.ballDetected                 = data.ballDetected
    #     self.state.homeDetected                 = data.homeDetected
    #     self.state.awayDetected                 = data.awayDetected
    #     self.state.our_bot_closest_to_ball      = data.our_bot_closest_to_ball
    #     self.state.opp_bot_closest_to_ball      = data.opp_bot_closest_to_ball
    #     self.state.our_goalie                   = data.our_goalie
    #     self.state.opp_goalie                   = data.opp_goalie
    #     self.state.opp_bot_marking_our_attacker = data.opp_bot_marking_our_attacker
    #     self.state.ball_at_corners              = data.ball_at_corners
    #     self.state.ball_in_our_half             = data.ball_in_our_half
    #     self.state.ball_in_our_possession       = data.ball_in_our_possession

    #     self.isteamyellow = data.isteamyellow
    #     self.pos = data.homePos[self.kubs_id]
    #     # self.vx = data.homeVel[self.kubs_id].x
    #     # self.vy = data.homeVel[self.kubs_id].y
    #     self.c=self.c+1
    #     print(self.state.ballPos.x, self.state.ballPos.y)
    #     print("shubham   "+str(self.c))

    #     #print(str(self.state.ballPos.x))
    #     #if data.homeDetected[self.kubs_id] == True:
    #         #print("kubs_id " + str(self.kubs_id) + "Detected")
    #     #else:
    #         #print("kubs_id " + str(self.kubs_id) + "Not Detected")

    # def kubsBelief(self):
    #     #rospy.init_node('kubs_node', anonymous=False)
    #     rospy.Subscriber('/belief_state',BeliefState,self.bs_callback,queue_size=1000)
       # rospy.spin()