import behavior
from enum import Enum
import config

class Dribble(behavior.Behavior):

    class State(Enum):
        setup = 1
        behind = 2
        near = 3
        drive = 4

    def __init__():
        super().__init__():
        for state in Dribble.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Dribble.State.setup, lambda: True)
        self.add_transition(Dribble.State.setup, Dribble.State.behind, lambda: isSetupCompleted())
        self.add_transition(Dribble.State.behind, Dribble.State.near, lambda: isMoveCompleted())
        self.add_transition(Dribble.State.near, Dribble.State.drive, lambda: isBallNearTheKub())
        self.add_transition(Dribble.State.drive, behavior.Behavior.State.completed, lambda: isDriveCompleted())

    def on_enter_setup(self):
        #move_point = getPointBehindTheBall(target_point)

    def execute_setup(self):
        pass

    def on_exit_setup(self):
        pass

    def on_enter_behind(self):
        pass

    def execute_behind(self):
        pass

    def on_exit_behind(self):
        pass

    def on_enter_near(self):
        pass

    def execute_near(self):
        pass

    def on_exit_near(self):
        pass

    def on_enter_drive(self):
        pass

    def execute_drive(self):
        pass

    def on_exit_drive(self):
        pass

