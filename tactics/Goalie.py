import behavior
from role import GoToPoint,GoToBall
import enum
import math
from utils.functions import *
from utils.config import *
import memcache
import os
shared = memcache.Client(['127.0.0.1:11211'],debug=False)


class Goalie(behavior.Behavior):

    MIN_VEL = 10.0

    class State(enum.Enum):
        ## Normal gameplay, stay towards the side of the goal that the ball is on.
        protect = 1
        ## Get the ball out of our defense area.
        clear = 2
        ## Keep calm and wait for the ball to be valid.
        peace = 3

    def __init__(self,continuous=False):
        super(Goalie,self).__init__()

        for substate in Goalie.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Goalie.State.peace, lambda: True, "immediately")

        self.add_transition(Goalie.State.peace,
            Goalie.State.protect, lambda: self._peace_to_protect(), "ball is valid")

        self.add_transition(Goalie.State.protect,
                            Goalie.State.clear, lambda: self._protect_to_clear(),
                        "save now")
        

    def GoToPoint(self,point,theta):
        pass

    def add_kub(self,kub):
        self.kub = kub

    def _peace_to_protect(self):
        state = shared.get('state')
        return (state.ballVel.x < -self.MIN_VEL or state.ballPos.x<=0) 

    def _any_to_peace(self):
        state = shared.get('state')
        return (state.ballVel.x >= 0 and state.ballPos.x>=0)

    def _protect_to_clear(self):
        state = shared.get('state')
        return (state.ballPos.x < -HALF_FIELD_MAXX + OUR_GOAL_MAXX)

    # note that execute_running() gets called BEFORE any of the execute_SUBSTATE methods gets called

    def execute_peace(self):
        if self.kub != None:
            self.GTP = GoToPoint.GoToPoint()
            self.GTP.add_kub(self.kub)
            self.GTP.add_point(Vector2D(-HALF_FIELD_MAXX,0),0)
            self.GTP.spin()

    def execute_clear(self):
        self.gtB = GoToBall.GoToBall(execute_fine_approach = False)
        self.gtB.add_kub(self.kub)
        state = shared.get('state')
        self.gtB.add_theta(theta=normalize_angle(math.pi + angle_diff(state.ballPos,state.homePos[self.kub.kubs_id])))
        self.gtB.spin()

    def execute_protect(self):
        state = shared.get('state')
        expected_y = goalie_expected_y(state, self.kub.kubs_id)
        self.GTP = GoToPoint.GoToPoint()
        self.GTP.add_kub(self.kub)
        if (expected_y > 0):
            self.GTP.add_point(Vector2D(-HALF_FIELD_MAXX,min(expected_y,OUR_GOAL_MAXY)), angle_diff(self.kub.get_pos(),state.ballPos))
        else:
            self.GTP.add_point(Vector2D(-HALF_FIELD_MAXX,max(expected_y,OUR_GOAL_MINY)), angle_diff(self.kub.get_pos(),state.ballPos))
        self.GTP.spin()

