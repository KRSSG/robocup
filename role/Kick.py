import behavior
import enum
from utils.math_functions import *
from utils.config import *

## lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
# Note: LineKick recalculates the aim_target_point ONLY when the target point/segment changes
#
# See Also: LineKickOld is the old, python-only implementation of line_kick
class Kick(behavior.Behavior):
	
    class State(enum.Enum):
        waiting = 1  # waiting state does nothing
        kick = 2

    def __init__(self):
        super().__init__()

        self._got_close = False
        self.kub = None

        self.add_state(LineKick.State.waiting, behavior.Behavior.State.running)
        self.add_state(LineKick.State.kick, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            LineKick.State.waiting, lambda: True,
                            'immediately')
        self.add_transition(LineKick.State.waiting, LineKick.State.kick,
                            lambda: self.enable_kick, 'kicker is enabled')

        self.add_transition(
            LineKick.State.kick, behavior.Behavior.State.completed,
            lambda: self.kub is not None and self._got_close,
            "robot kicked")

    def add_kub(self,kub):
    	self.kub = kub


    @property
    def kick_power(self):
        return self._kick_power

    @kick_power.setter
    def kick_power(self, value):
        self._kick_power = value

    @property
    def chip_power(self):
        return self._chip_power

    @chip_power.setter
    def chip_power(self, value):
        self._chip_power = value

    ## If false, uses straight kicker, if true, uses chipper (if available)
    # Default: False
    @property
    def use_chipper(self):
        return self._use_chipper

    @use_chipper.setter
    def use_chipper(self, value):
        self._use_chipper = value

    ## If set to False, will get all ready to go, but won't kick/chip just yet
    # Can be used to synchronize between behaviors
    # Default: True
    @property
    def enable_kick(self):
        return self._enable_kick

    @enable_kick.setter
    def enable_kick(self, value):
        self._enable_kick = value

    def execute_kick(self):
        # To implement in path planner
        ## self.robot.disable_avoid_ball()

        if dist(self.kub.get_pos(),self.kub.ballPos) < BOT_BALL_THRESH:
            self._got_close = True

        if self._got_close:
            if self.use_chipper:
            	pass
            else:
                self.kub.kick(self.chip_power)
        self.kub.execute()

    
