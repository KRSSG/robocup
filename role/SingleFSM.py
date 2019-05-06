print("Gotopoint imported")
from enum import Enum
import behavior
print("Importing _gotopoint_")
import _GoToPoint_
try:
    _GoToPoint_ = reload(_GoToPoint_)
except:
    import _GoToPoint_
import rospy
import math
from utils.functions import *
from utils.config import *

#FIRST_CALL = True

class SingleFSM(behavior.Behavior):
    """docstring for SingleFSM"""
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        in_left = 2
        in_right = 3


    def __init__(self,continuous=False):
        # print "gtp"
        #SingleFSM.behavior.Behavior()
        #g = behavior.Behavior()
        #print "gtp2"
        super(SingleFSM,self).__init__()
        #self.state = state
        self.name = "SingleFSM"

        self.behavior_failed = False
        self.DISTANCE_THRESH = DISTANCE_THRESH

        self.add_state(SingleFSM.State.in_left, behavior.Behavior.State.running)
        self.add_state(SingleFSM.State.in_right, behavior.Behavior.State.running)
        

        self.add_transition(behavior.Behavior.State.start, SingleFSM.State.in_left,lambda: self.check_half(),'start_left')
        self.add_transition(behavior.Behavior.State.start, SingleFSM.State.in_right,lambda: not self.check_half(),'start_right')

        self.add_transition(SingleFSM.State.in_left, SingleFSM.State.in_right,lambda: not self.check_half(),'moved right of halfline')
        self.add_transition(SingleFSM.State.in_right, SingleFSM.State.in_left, lambda: self.check_half(),'moved left of halfline')

    def add_kub(self,kub):
        self.kub = kub
        
    def check_half(self):
        if self.kub.state.homePos[self.kub.kubs_id].x < 0 :
            return True
        
    def on_enter_in_left(self):
        print("Entered on_enter_in_left()")
        # global FIRST_CALL
        # if FIRST_CALL:
        self.target_point = Vector2D()
        self.target_point.x = 3000
        self.target_point.y = 0
        self.theta = 0
        self.start_time = rospy.Time.now()
        self.start_time = 1.0*self.start_time.secs + 1.0*self.start_time.nsecs/pow(10,9)
            #FIRST_CALL = False
        _GoToPoint_.init(self.kub,self.target_point,self.theta)
        pass

    def terminate(self):
        super().terminate()

    def execute_in_left(self):
        print("Execute in_left")
        generatorFunction = _GoToPoint_.execute(self.start_time,self.DISTANCE_THRESH)
        prev_x = self.kub.state.homePos[self.kub.kubs_id].x
        prev_y = self.kub.state.homePos[self.kub.kubs_id].y
        prev_theta = self.kub.state.homePos[self.kub.kubs_id].theta

        print "x=", prev_x, ", y=", prev_y, ", theta=", prev_theta
        for gf in generatorFunction :
            self.kub, GOAL_POINT = gf
            if math.sqrt(math.pow(self.kub.state.homePos[self.kub.kubs_id].x-prev_x,2)+math.pow(self.kub.state.homePos[self.kub.kubs_id].y-prev_y,2)) > BOT_BALL_THRESH/2 :
                break

        print("after execute in_left")
    
    def on_exit_in_left(self):
        # global FIRST_CALL
        # FIRST_CALL = True
        print("on_exit_in_left")
        pass

    def on_enter_in_right(self):
        # global FIRST_CALL
        # if FIRST_CALL:
        self.target_point = Vector2D()
        self.target_point.x = -3000
        self.target_point.y = 0
        self.theta = 0
        self.start_time = rospy.Time.now()
        self.start_time = 1.0*self.start_time.secs + 1.0*self.start_time.nsecs/pow(10,9)
        #FIRST_CALL = False
        _GoToPoint_.init(self.kub,self.target_point,self.theta)
        pass

    def execute_in_right(self):
        print("Execute in_right")
        generatorFunction = _GoToPoint_.execute(self.start_time,self.DISTANCE_THRESH)
        prev_x = self.kub.state.homePos[self.kub.kubs_id].x
        prev_y = self.kub.state.homePos[self.kub.kubs_id].y
        prev_theta = self.kub.state.homePos[self.kub.kubs_id].theta

        print "x=", prev_x, ", y=", prev_y, ", theta=", prev_theta
        for gf in generatorFunction :
            self.kub, GOAL_POINT = gf
            if math.sqrt(math.pow(self.kub.state.homePos[self.kub.kubs_id].x-prev_x,2)+math.pow(self.kub.state.homePos[self.kub.kubs_id].y-prev_y,2)) > BOT_BALL_THRESH/2 :
                break
    
    def on_exit_in_right(self):
        # global FIRST_CALL
        # FIRST_CALL = True
        pass




