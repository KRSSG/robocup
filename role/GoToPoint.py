from enum import Enum
import behavior
import _GoToPoint
import rospy
from utils.math_functions import *
from utils.config import *

class GoToPoint(behavior.Behavior):
    """docstring for GoToPoint"""
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        setup = 1 
        drive = 2


    def __init__(self,continuous=False):
        # print "gtp"
        #GoToPoint.behavior.Behavior()
        #g = behavior.Behavior()
        #print "gtp2"
        super(GoToPoint,self).__init__()
        #self.state = state

        self.name = "GoToPoint"

        self.behavior_failed = False

        self.add_state(GoToPoint.State.setup,
            behavior.Behavior.State.running)
        self.add_state(GoToPoint.State.drive,
            behavior.Behavior.State.running)
        

        self.add_transition(behavior.Behavior.State.start,
            GoToPoint.State.setup,lambda: True,'immediately')

        self.add_transition(GoToPoint.State.setup,
            GoToPoint.State.drive,lambda: self.target_present,'setup')

        self.add_transition(GoToPoint.State.drive,
            GoToPoint.State.drive,lambda: not self.at_new_point(),'restart')

        self.add_transition(GoToPoint.State.drive,
            behavior.Behavior.State.completed,lambda:self.at_new_point(),'complete')

        self.add_transition(GoToPoint.State.setup,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        self.add_transition(GoToPoint.State.drive,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')


    def add_point(self,point,orient=None):
        self.target_point = point
        if orient:
            self.theta = orient
        else:
            self.theta = self.kub.get_pos().theta
        
    def add_kub(self,kub):
        self.kub = kub
        
    def target_present(self):
        return self.target_point is not None


    def at_new_point(self):
        #print (dist(self.target_point,self.new_point),210)
        return dist(self.target_point,self.new_point) < DISTANCE_THRESH

        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        _GoToPoint.init(self.kub,self.target_point,self.theta)
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_drive(self):
        pass

    def terminate(self):
        super().terminate()

    def execute_drive(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint.execute(start_time,DISTANCE_THRESH)
        for gf in generatingfunction:
            self.kub,target_point = gf

            # print self.behavior_failed
            if not vicinity_points(self.target_point,target_point):
                # print 
                # print  (self.target_point.x,self.target_point.y)
                # print  (target_point.x,target_point.y)
                # print 
                self.behavior_failed = True
                # print self.behavior_failed
                break
        self.new_point = self.kub.get_pos()
        

    
    def on_exit_drive(self):
        pass




