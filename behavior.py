from enum import Enum
import fsm
import logging


## Behavior is an abstract superclass for Skill, Play, etc
class Behavior(fsm.StateMachine):

    ## These are the core states of the Behavior class
    # Subclasses may extend this by adding substates of the following
    class State(Enum):
        start = 1
        running = 2
        completed = 3

        # note: these two states are only relevant for non-continuous behaviors
        failed = 4
        cancelled = 5

    def __init__(self):
        # print "behav"
        super(Behavior, self).__init__(start_state=Behavior.State.start,end_state=Behavior.State.completed)
        # add base states for Behavior
        self.add_state(Behavior.State.start)
        self.add_state(Behavior.State.running)
        self.add_state(Behavior.State.completed)
        self.add_state(Behavior.State.failed)
        self.add_state(Behavior.State.cancelled)

        #self._is_continuous = continuous

    # def add_state(self, state, parent_state=None):
    #     super().add_state(state, parent_state)
        #TODO: raise exception if @state doesn't have a Behavior.State ancestor

        ## Whether or not the Behavior is running
        # Because we use hierarchial state machines, a behavior never be in the "running", but may be in a substate of it
        # This is a convenience method to check whether or not the play is running
    def is_done_running(self) :
        for state in [Behavior.State.completed, Behavior.State.failed,
                      Behavior.State.cancelled]:
            if self.is_in_state(state): return True

        return False

    ## Set the behavior to failed if sub behaviors remains uncompleted
    def set_failed(self):
        if self.is_done_running():
            logging.warn("Attempt to set fail flag to a behavior that's already done running")
        else:
            self.transition(Behavior.State.failed)

    ## Transitions the Behavior into a terminal state (either completed or cancelled)
    def terminate(self):
        if self.is_done_running():
            logging.warn("Attempt to terminate behavior that's already done running")
        else:
            self.transition(Behavior.State.cancelled)

    ## returns a state in Behavior.State that represents what the behaviors is doing
    # use this instead of the @state property if you want to avoid dealing with custom subclass substates
    @property
    def behavior_state(self):
        return self.corresponding_ancestor_state(list(Behavior.State))


