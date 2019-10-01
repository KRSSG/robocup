import composite_behavior
import behavior
# import robocup
from utils.config import *
# import main
import enum
import math
import time
import rospy,sys
# import role_assignment
# import skills
from utils.functions import *
from role import GoToPoint
from krssg_ssl_msgs.srv import *

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

# PassReceive accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise
# it will go to 'failed'.
class PassReceive(composite_behavior.CompositeBehavior):

    # max difference between where we should be facing and where we are facing
    # (in radians)
    FaceAngleErrorThreshold = deg_2_radian(10.0)

    # how much we're allowed to be off in the direction of the pass line
    PositionYErrorThreshold = BOT_RADIUS

    # how much we're allowed to be off side-to-side from the pass line
    PositionXErrorThreshold = BOT_RADIUS

    # we have to be going slower than this to be considered 'steady'
    SteadyMaxVel = MIN_BOT_SPEED
    SteadyMaxAngleVel = MIN_BOT_OMEGA

    StabilizationFrames = 3
    DesperateTimeout = 5

    class State(enum.Enum):
        # we're aligning with the planned receive point
        aligning = 1

        # being in this state signals that we're ready for the kicker to kick
        aligned = 2

        # the ball's been kicked and we're adjusting based on where the ball's
        # moving
        waiting = 3
        receiving = 4

    def __init__(self, captureFunction=(lambda: skills.capture.Capture())):
        super(PassReceive,self).__init__()
        # super().__init__(continuous=False,
        #                  # Don't restart play if we change robots while kicking
        #                  # the ball
        #                  autorestart=lambda: not self.ball_kicked)

        self.ball_kicked = False
        self._target_pos = None
        self._receive_point = None
        self.captureFunction = captureFunction

        for state in PassReceive.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            PassReceive.State.aligning, lambda: True,
                            'immediately')

        self.add_transition(
            PassReceive.State.aligning, PassReceive.State.aligned,
            lambda: self.errors_below_thresholds() and self.is_steady() and not self.ball_kicked,
            'steady and in position to receive')

        self.add_transition(
            PassReceive.State.aligned, PassReceive.State.aligning,
            lambda: (not self.errors_below_thresholds()
                     or not self.is_steady()) and not self.ball_kicked,
            'not in receive position')

        for state in [PassReceive.State.aligning, PassReceive.State.aligned]:
            self.add_transition(state, PassReceive.State.waiting, lambda:self.ball_kicked and not self.is_ball_near(), 'waiting')
            self.add_transition(state, PassReceive.State.receiving,
                                lambda: self.ball_kicked and self.is_ball_near(), 'ball near bot, move backward with dribbler')

        self.add_transition(PassReceive.State.waiting, PassReceive.State.receiving, lambda: self.is_ball_near(), 'ball near bot, move backward with dribbler')
        self.add_transition(PassReceive.State.receiving,
                            behavior.Behavior.State.completed,
                            lambda: self.kub.has_ball(), 'ball received!')

        self.add_transition(
            PassReceive.State.receiving, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name(
                'capture').state == behavior.Behavior.State.failed or self.check_failure(),
            'ball missed :(')

    # set this to True to let the receiver know that the pass has started and the ball's in motion
    # Default: False
    @property
    def ball_kicked(self):
        return self._ball_kicked

    @ball_kicked.setter
    def ball_kicked(self, value):
        self._ball_kicked = value
        if value:
            self._ball_kick_time = time.time()

    def add_kub(self, kub):
        self.kub = kub

    # The point that the receiver should expect the ball to hit it's mouth
    # Default: None
    @property
    def receive_point(self):
        return self._receive_point

    @receive_point.setter
    def receive_point(self, value):
        self._receive_point = value
        self.recalculate()

    # returns True if we're facing the right direction and in the right
    # position and steady
    def errors_below_thresholds(self):
        if not isinstance(self.receive_point, Vector2D):
            return False

        return (
            abs(self._angle_error) < PassReceive.FaceAngleErrorThreshold and
            abs(self.prep_error) < PassReceive.PositionXErrorThreshold and
            abs(self.along_line_error) < PassReceive.PositionYErrorThreshold)

    def is_steady(self):
        kub_vel = self.kub.get_vel()
        return kub_vel['magnitude'] < PassReceive.SteadyMaxVel and self.kub.vw < PassReceive.SteadyMaxAngleVel


    def is_ball_near(self):
        return True
        kub_pos = Vector2D(self.kub.get_pos())
        ball_pos = Vector2D(self.kub.state.ballPos)
        print("Distance from ball ",ball_pos.dist(kub_pos))
        return ball_pos.dist(kub_pos) < 5*BOT_RADIUS
    # calculates:
    # self._pass_line - the line from the ball along where we think we're going
    # self._target_pos - where the bot should be
    # self._angle_error - difference in where we're facing and where we want to face (in radians)
    # self._x_error
    # self._y_error
    def recalculate(self):
        # can't do squat if we don't know what we're supposed to do
        if not isinstance(self.receive_point, Vector2D) or self.kub == None:
            return

        kub_pos = Vector2D(self.kub.get_pos())
        ball_pos = Vector2D(self.kub.state.ballPos)
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        ball_angle = ball_vel.angle()
        if self.ball_kicked:
            # when the ball's in motion, the line is based on the ball's
            # velocity
            self.pass_line = Line(point1=ball_pos, angle=ball_angle)
        else:
            # if the ball hasn't been kicked yet, we assume it's going to go
            # through the receive point
            self.pass_line = Line(point1=ball_pos, point2=self.receive_point)

        target_angle_rad = kub_pos.angle(ball_pos)
        angle_rad = self.kub.get_pos().theta
        self._angle_error = target_angle_rad - angle_rad

        if self.ball_kicked:
            actual_receive_point = self.pass_line.nearest_point(kub_pos)
        else:
            actual_receive_point = self.receive_point

        pass_line_normalized_vector = self.pass_line.normalized_vector()
        self._target_pos = actual_receive_point + pass_line_normalized_vector*BOT_RADIUS

        pos_error = self._target_pos - kub_pos
        self.prep_error = self.pass_line.distance_from_point(kub_pos)
        self.along_line_error = math.sqrt(
            self._target_pos.distSq(kub_pos) - self.prep_error**2)

    def on_exit_start(self):
        # reset
        self.ball_kicked = False

    # def execute_running(self):
    #     # pass
    #     state = shared.get('state')
    #     self.kub.update_state(state)
    #     kub_pos = Vector2D(self.kub.get_pos())
    #     face_angle = self.pass_line.angle + math.pi
    #     self.recalculate()
    #     self.face = GoToPoint.GoToPoint()
    #     self.face.add_kub(self.kub)
    #     self.face.add_point(point=kub_pos, orient=face_angle)
    #     self.add_subbehavior(self.face, 'face')

    def execute_waiting(self):
        print("execute waiting")
        self.kub.dribble(True)
        self.kub.execute()

    def execute_aligning(self):
        print("execute aligning")
        self.recalculate()
        if isinstance(self._target_pos,Vector2D):
            move_angle = self.pass_line.angle + math.pi
            self.move = GoToPoint.GoToPoint()
            try:
                state = getState(state).stateB
            except rospy.ServiceException, e:
                print("Error ",e)           
            self.kub.update_state(state)
            self.move.add_kub(self.kub)
            print(move_angle)
            self.move.add_point(point=self._target_pos, orient=move_angle)
            self.add_subbehavior(self.move, 'move')


    def execute_aligned(self):
        print("execute aligning")
        try:
            state = getState(state).stateB
        except rospy.ServiceException, e:
            print("Error ",e) 
        self.recalculate()
        face_angle = self.pass_line.angle + math.pi
        self.face = GoToPoint.GoToPoint()
        self.kub.update_state(state)
        point = self.kub.get_pos()
        self.face.add_kub(self.kub)
        self.face.add_point(point)
        self.add_subbehavior(self.face, 'face')
    def on_enter_receiving(self):
        pass

    def on_exit_receiving(self):
        pass
        raise NotImplementedError

    def check_failure(self):
        THRESH = 1.5 * BOT_RADIUS
        ball_pos = Vector2D(self.kub.state.ballPos)
        kub_pos = Vector2D(self.kub.get_pos.x, self.kub.get_pos.y)
        pass_line = self.pass_line
        ball_line_distance = pass_line.distance_from_point(ball_pos)
        kick_angle = pass_line.angle
        bot_ball_angle = kub_pos.angle(ball_pos)

        opp_kick_angle = Vector2D().normalizeAngle(kick_angle + math.pi)
        diff_angle = Vector2D().normalizeAngle(opp_kick_angle - bot_ball_angle)
        if fabs(diff_angle) < math.pi / 2 and ball_line_distance < THRESH:
            return False
        else:
            return True

    def execute_receiving(self):
        try:
                state = getState(state).stateB
        except rospy.ServiceException, e:
                print("Error ",e) 
        velocity_magnitude = MAX_BOT_SPEED/(5*BOT_RADIUS)
        kub_angle = self.pass_line.angle
        vx = velocity_magnitude*math.cos(kub_angle)
        vy = velocity_magnitude*math.sin(kub_angle)
        self.kub.move(vx, vy)
        self.kub.dribble(True)
        self.kub.execute()
        # ################## TODO:
        # WRITE DRIBBLE TACTIC
        #######################################
        # self.catch = Dribble.Dribble()
        # self.catch.add_kub(self.kub)
        # raise NotImplementedError
