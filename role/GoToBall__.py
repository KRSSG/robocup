from enum import Enum
import behavior
import _GoToPoint
import rospy
from utils.math_functions import *
from utils.config import *

class GoToBall(behavior.Behavior):
    """docstring for GoToBall"""
    class State(Enum):
        setup = 1 
        course_approach = 2
        fine_approach = 3

    def __init__(self,continuous=False):

        super(GoToBall,self).__init__()
        # self.kub = kub

        self.name = "GoToBall"

    	self.power = 7.0

        self.initial_target_dist_thresh = DISTANCE_THRESH/3
        self.ball_dist_thresh = BOT_BALL_THRESH

        self.behavior_failed = False

        self.add_state(GoToBall.State.setup,
            behavior.Behavior.State.running)

        self.add_state(GoToBall.State.course_approach,
            behavior.Behavior.State.running)
        
        self.add_state(GoToBall.State.fine_approach,
            behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            GoToBall.State.setup,lambda: True,'immediately')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.fine_approach,lambda: self.ball_in_vicinity(),'ball_in_vicinity')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.course_approach,lambda: self.target_present(),'setup')

        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.fine_approach,lambda:self.at_target_point(),'complete')

        self.add_transition(GoToBall.State.fine_approach,
            behavior.Behavior.State.completed,lambda:self.at_ball_pos(),'complete')

        self.add_transition(GoToBall.State.setup,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        self.add_transition(GoToBall.State.course_approach,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        self.add_transition(GoToBall.State.fine_approach,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
    
    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta
    
    def target_present(self):
        return not ball_in_front_of_bot(self.kub) and self.target_point is not None

    def at_target_point(self):
        # print (self.target_point.x,self.target_point.y,self.kub.get_pos(),210)
        # print "at tp",dist(self.target_point,self.kub.get_pos()) , DISTANCE_THRESH,self.kub.get_pos().x,self.kub.get_pos().y
        # print "at tp",self.target_point.x,self.target_point.y
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.initial_target_dist_thresh)

    def ball_in_vicinity(self):
        # print "ibp",ball_in_front_of_bot(self.kub)
        if ball_in_front_of_bot(self.kub):
            self.target_point = None
            return True
        return False

    def at_ball_pos(self):
        error = 10
        # print "at bp",dist(self.kub.state.ballPos,self.kub.get_pos()) , DISTANCE_THRESH/2
        # print (self.kub.state.ballPos.x,self.kub.state.ballPos.y,self.kub.get_pos(),210)
        return vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=self.ball_dist_thresh+error) 

    def terminate(self):
        super().terminate()
        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        # print self.target_point.x,self.target_point.y
        _GoToPoint.init(self.kub, self.target_point, self.theta)
        pass
        
    def on_exit_setup(self):
        
        # print ball_in_front_of_bot(self.kub)
        pass

    def on_enter_course_approach(self):
        pass

    def execute_course_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint.execute(start_time,self.initial_target_dist_thresh)
        for gf in generatingfunction:
            self.kub,target_point = gf
            self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            # print ball_in_front_of_bot(self.kub)
            # print (self.behavior_failed,(self.target_point.x,self.target_point.y))
            # print (self.behavior_failed,(target_point.x,target_point.y))

            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*3.5):
                self.behavior_failed = True
                break


    def on_exit_course_approach(self):
        
        pass

    def on_enter_fine_approach(self):
        theta = self.kub.get_pos().theta
        _GoToPoint.init(self.kub, self.kub.state.ballPos, theta)
        pass

    def execute_fine_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint.execute(start_time,self.ball_dist_thresh)
        for gf in generatingfunction:
            self.kub,ballPos = gf
            
            # print (self.behavior_failed,(self.kub.state.ballPos.x,self.kub.state.ballPos.y))
            if not vicinity_points(ballPos,self.kub.state.ballPos,thresh=BOT_RADIUS):
                self.behavior_failed = True
                break




    def disable_kick(self):
        self.power = 0.0

    def on_exit_fine_approach(self):
        
        self.kub.kick(self.power)
        self.kub.execute()
        pass





