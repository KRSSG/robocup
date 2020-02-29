from enum import Enum
import behavior
import _GoToPoint_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from math import *
import time
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.srv import bsServer
from krssg_ssl_msgs.srv import *
from krssg_ssl_msgs.msg import Ref
first = 0
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)
state = None

class GoToBall(behavior.Behavior):
    """docstring for GoToBall"""
    class State(Enum):
        setup = 1 
        course_approach = 2
        fine_approach = 3
        intercept = 4

    def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

        super(GoToBall,self).__init__()

        self.name = "GoToBall"

        self.power = 7.0

        self.target_point = None
        

        self.course_approch_thresh = course_approch_thresh

        self.ball_dist_thresh = 2*BOT_BALL_THRESH

        self.behavior_failed = False


        self.add_state(GoToBall.State.setup,
            behavior.Behavior.State.running)

        self.add_state(GoToBall.State.course_approach,
            behavior.Behavior.State.running)
        
        self.add_state(GoToBall.State.fine_approach,
            behavior.Behavior.State.running)

        self.add_state(GoToBall.State.intercept,
            behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            GoToBall.State.setup,lambda: True,'immediately')

        self.add_transition(GoToBall.State.setup,
           GoToBall.State.intercept,lambda:self.ball_moving(),'intercept_ball')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.fine_approach,lambda: self.fine_approach() and not self.ball_moving() ,'ball_in_vicinity')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.course_approach,lambda: self.course_approach() and not self.ball_moving(),'setup')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.intercept,lambda:self.ball_moving(),'intercept_ball')

        self.add_transition(GoToBall.State.intercept,
            GoToBall.State.course_approach,lambda:self.course_approach() and not self.ball_moving(),'setup')

        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.fine_approach,lambda:self.at_target_point() ,'complete')

        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.intercept,lambda:self.ball_moving(),'intercept_ball')

        #self.add_transition(GoToBall.State.fine_approach,
           #GoToBall.State.intercept,lambda:self.ball_moving(),'intercept_ball')

        self.add_transition(GoToBall.State.fine_approach,
            behavior.Behavior.State.completed,lambda:self.at_ball_pos(),'complete')

        self.add_transition(GoToBall.State.intercept,
            GoToBall.State.fine_approach,lambda:self.intercept_complete(),'intercept_complete')

        # self.add_transition(GoToBall.State.setup,
        #   behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
        #These three conditions for fail might cause a problem in dynamic gameplay as we are sending it back to setup and going to new point.
        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(GoToBall.State.fine_approach,
            GoToBall.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(GoToBall.State.intercept,
            GoToBall.State.setup,lambda: self.behavior_failed,'failed')

    
    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta

    def fine_approach(self):
        return self.ball_in_vicinity() 

    def course_approach(self):
        return not self.ball_in_vicinity() 
    # def target_present(self):
    #     return not ball_in_front_of_bot(self.kub) and self.target_point is not None 

    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.course_approch_thresh)


    def ball_in_vicinity(self):
        if ball_in_front_of_bot(self.kub):
            return True
        return False

    def ball_moving(self):
        #print("try to move idiot")
        return False
        #print("vx = ",self.kub.state.ballVel.x
        ball_vel_dir = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        ball_vel_angle = ball_vel_dir.tan_inverse()
        bot_ball = Vector2D(self.kub.state.ballPos.x-self.kub.state.homePos[self.kub.kubs_id].x , self.kub.state.ballPos.y-self.kub.state.homePos[self.kub.kubs_id].y)
        bot_ball_angle = bot_ball.tan_inverse()
        perp_dist = sqrt((self.kub.state.homePos[self.kub.kubs_id].x -self.kub.state.ballPos.x)**2 + (self.kub.state.homePos[self.kub.kubs_id].y -self.kub.state.ballPos.y)**2)*sin(abs(ball_vel_angle-bot_ball_angle))
        #if ( getTime(perp_dist) < )
        if(abs(ball_vel_angle-bot_ball_angle) < SATISFIABLE_THETA_DEG):
            return False
        if (abs(self.kub.state.ballVel.x) > 10*MIN_BOT_SPEED and abs(self.kub.state.ballVel.x) < MAX_BOT_SPEED ) or ( abs(self.kub.state.ballVel.y) > 10*MIN_BOT_SPEED and abs(self.kub.state.ballVel.y) < MAX_BOT_SPEED ) :
            print("ball moved", self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            return True
        else:
            return False

    def at_ball_pos(self):
        error = 50
        return vicinity_points(self.kub.get_pos(),self.kub.state.ballPos,thresh=self.ball_dist_thresh+error) 

    def terminate(self):
        super().terminate()
        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_course_approach(self):
        global state
        state = None
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        self.target_point = self.kub.state.ballPos
        try:
            state = getState(state).stateB    
        except rospy.ServiceException, e:
            print("chutiya")        
        if state :
            self.theta = normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000))
        _GoToPoint_.init(self.kub, self.target_point, self.theta)
        pass

    def execute_course_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        for gf in generatingfunction:
            self.kub,target_point = gf
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            self.target_point = self.kub.state.ballPos
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                break


    def on_exit_course_approach(self):
        pass

    def on_exit_intercept(self):
        pass

    def on_enter_fine_approach(self):
        theta = self.kub.get_pos().theta
        _GoToPoint_.init(self.kub, self.kub.state.ballPos, theta)
        pass

    def execute_fine_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh)
        for gf in generatingfunction:
            self.kub,ballPos = gf
            
            if not vicinity_points(ballPos,self.kub.state.ballPos,thresh=BOT_RADIUS):
                self.behavior_failed = True
                break

    def intercept_complete(self):
        ball_vel = Vector2D(self.kub.state.ballVel.y,self.kub.state.ballVel.x)
        ball_vel_angle = ball_vel.tan_inverse()
        bot_ball_dir = Vector2D(self.kub.state.ballPos.y-self.kub.state.homePos[self.kub.kubs_id].y , self.kub.state.ballPos.x-self.kub.state.homePos[self.kub.kubs_id].x)
        if ( abs(ball_vel_angle - bot_ball_dir.tan_inverse() )< 0.0523599):
            return 1
        return 0

    # def intercept_possible(self,angle,ball_vel_angle,bot_ball_angle,perp_dist):
        
    #   ball_dist = (perp_dist*tan(1.5707963-(3.14159265-ball_vel_angle+bot_ball_angle)) - perp_dist*tan(angle) )
    #   a = cos(angle)
    #   if a == 0:
    #       a = 0.0001
    #   if self.ball_moving():
    #       if(1.5*self.getTime(perp_dist/a) < ball_dist/sqrt(self.kub.state.ballVel.y*self.kub.state.ballVel.y + self.kub.state.ballVel.x*self.kub.state.ballVel.x) ):
    #           return True
    #       return False
    #   return False

    def on_enter_intercept(self):
        global first
        first = 1

    def execute_intercept(self):
        #print("ballvel= ",state.ballVel)
        #$print("intercept")
        global first
        ball_vel = Vector2D(self.kub.state.ballVel.y,self.kub.state.ballVel.x)
        bot_ball_dir = Vector2D(self.kub.state.ballPos.y-self.kub.state.homePos[self.kub.kubs_id].y , self.kub.state.ballPos.x-self.kub.state.homePos[self.kub.kubs_id].x)
        ball_vel_angle = ball_vel.tan_inverse()
        bot_ball_angle = bot_ball_dir.tan_inverse()
        perp_dist = sqrt((self.kub.state.homePos[self.kub.kubs_id].x -self.kub.state.ballPos.x)**2 + (self.kub.state.homePos[self.kub.kubs_id].y -self.kub.state.ballPos.y)**2)*sin(abs(ball_vel_angle-bot_ball_angle))
        approach = True
        
        if (approach ):
            first = 0
            #theta = 1.5707963-abs(ball_vel_angle-bot_ball_angle) - 0.0872665   
            my_target = Vector2D(0,0)   
                
            final_theta = atan2(self.kub.state.ballVel.x,self.kub.state.ballVel.y)

            x_final = self.kub.state.homePos[self.kub.kubs_id].x + perp_dist*cos(final_theta)
            y_final = self.kub.state.homePos[self.kub.kubs_id].y + perp_dist*sin(final_theta)
            my_target = Vector2D(x_final, y_final)          
                
            start_time = rospy.Time.now()
            start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
            print("my_target = ",my_target)
            _GoToPoint_.init(self.kub, my_target, 0)
            generatingfunction = _GoToPoint_.execute(start_time, self.ball_dist_thresh)
            

    def disable_kick(self):
        self.power = 0.0

    def on_exit_fine_approach(self):
        
        self.kub.kick(self.power)
        self.kub.execute()
        pass
