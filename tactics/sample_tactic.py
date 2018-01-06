from role import GoToBall,GoToPoint
from utils.math_functions import *
from utils.config import *
from utils.geometry import Vector2D


import composite_behavior
import behavior
import enum
import logging
from math import pi,atan2

# import memcache
# shared = memcache.Client(['127.0.0.1:11211'],debug=False)

class SampleTactic(composite_behavior.CompositeBehavior):

    class State(enum.Enum):
        preparing = 1  # the kicker is aiming and the receiver is getting ready
        running = 2  # waiting for the kicker to kick
        

    ## Init method for SampleTactic
    # @param skillreceiver an instance of a class that will handle the receiving robot. See pass_receive and angle_receive for examples.
    # Using this, you can change what the receiving robot does (rather than just receiving the ball, it can pass or shoot it).
    # Subclasses of pass_receive are preferred, but check the usage of this variable to be sure.
    # @param receive_point The point that will be kicked too. (Target point)
    # @param skillkicker A tuple of this form (kicking_class instance, ready_lambda). If none, it will use (pivot_kick lambda x: x == pivot_kick.State.aimed).
    # @param receiver_required Whether the receiver subbehavior should be required or not
    # @param kicker_required Whether the kicker subbehavior should be required or not
    # The lambda equation is called (passed with the state of your class instance) to see if your class is ready. Simple implementations will just compare it to your ready state.
    def __init__(self):
        super(SampleTactic,self).__init__()


        self.add_state(SampleTactic.State.preparing,
                       behavior.Behavior.State.running)
        
        self.add_state(SampleTactic.State.running,
                       behavior.Behavior.State.running)


        self.add_transition(behavior.Behavior.State.start,
                            SampleTactic.State.preparing, lambda: True,
                            'immediately')

        self.add_transition(
            SampleTactic.State.preparing, SampleTactic.State.running,
            lambda: True,
            'ready')

        # self.add_transition(
        #     SampleTactic.State.running, behavior.Behavior.State.completed,
        #     lambda: self.done_running(),
        #     'done!')

    # def done_running(self):
    #     return dist(self.kub.get_pos(),self.kub.state.ballPos)<DISTANCE_THRESH

    def add_kub(self,kub):
        self.kub = kub

    def on_enter_running(self):
        pass

    def execute_running(self):
        pass 

    def on_exit_running(self):
        self.remove_subbehavior('fetch')
        self.remove_subbehavior('move')
        pass


    def on_enter_preparing(self):
        # self.kub.state = shared.get('state')
        print (self.kub.get_pos().x,self.kub.get_pos().y)
        self.fetch = GoToBall.GoToBall()
        self.fetch.add_kub(self.kub)
        self.fetch.add_theta(theta=normalize_angle(pi+atan2(self.kub.state.ballPos.y,self.kub.state.ballPos.y-3000)))
        self.add_subbehavior(self.fetch,'fetch')


        # self.kub.state = shared.get('state')
        print (self.kub.get_pos().x,self.kub.get_pos().y)
        self.fetch = GoToBall.GoToBall()
        self.fetch.add_kub(self.kub)
        self.fetch.add_theta(theta=normalize_angle(pi+atan2(self.kub.state.ballPos.y,self.kub.state.ballPos.y-3000)))
        self.add_subbehavior(self.fetch,'move')

        print (self.kub.get_pos().x,self.kub.get_pos().y)
        # self.move = GoToPoint.GoToPoint()
        # self.move.add_kub(self.kub)
        # self.move.add_point(point=Vector2D(3000,0),orient=normalize_angle(pi))
        # self.add_subbehavior(self.move,'move')

        pass

    def execute_preparing(self):
        pass

    def on_exit_preparing(self):
        pass
        

