import behavior
import traceback
import logging
import sys
import time
from multiprocessing.dummy import Pool as ThreadPool

n_threads = 2

## A composite behavior is one that has 0+ named subbehaviors
# this class has methods for making it easy to work with and manage subbehaviors
class CompositeBehavior(behavior.Behavior):
    def __init__(self) :
        super(CompositeBehavior,self).__init__()
        self._subbehavior_info = {}  # type: Dict[str, Dict]

    # FIXME: what if a subbehavior of @bhvr is required, but this is not?
    # FIXME: how do priorities work?
    # FIXME: how do nested priorities work?
    
    ##
    ## @brief      Adds a subbehavior in composite behavior
    ##
    ## @param      bhvr      Behavior Instance
    ## @param      name      Name of Subbehavior, to be used for identification
    ## @param      required  TO BE IMPLEMENTED
    ## @param      priority  TO BE IMPLEMENTED
    ##
    ##
    def add_subbehavior(self,
                        bhvr,
                        name,
                        required=True,
                        priority=100):
        if name in self._subbehavior_info:
            raise AssertionError("There's already a subbehavior with name: '" +
                                 name + "'")

        if isinstance(priority, int):
            priority_func = (lambda: priority)
        else:
            priority_func = priority

        self._subbehavior_info[name] = {
            'required': required,
            'priority': priority_func,
            'behavior': bhvr
        }

    def remove_subbehavior(self, name):
        del self._subbehavior_info[name]

    def has_subbehavior_with_name(self, name):
        return name in self._subbehavior_info

    def has_subbehaviors(self) :
        return len(self._subbehavior_info) > 0

    def subbehavior_with_name(self, name):
        return self._subbehavior_info[name]['behavior']

    def subbehaviors_by_name(self) :
        by_name = {}
        for name in self._subbehavior_info:
            by_name[name] = self._subbehavior_info[name]['behavior']
        return by_name

    def remove_all_subbehaviors(self):
        subbehaviorNames = list(self._subbehavior_info.keys())
        for name in subbehaviorNames:
            self.remove_subbehavior(name)

    def all_subbehaviors(self) :
        return [
            self._subbehavior_info[name]['behavior']
            for name in self._subbehavior_info
        ]

    def all_subbehaviors_completed(self):
        return all(
            [bhvr.is_done_running() for bhvr in self.all_subbehaviors()])

    ## Override StateMachine.spin() so we can call spin() on subbehaviors

    ##
    ## @brief      Spin Composite Behavior,execute all behaviors inside composite behavior
    ##
    ## All behavior inside Composite behavior instance is executed using threading.
    ## If subbehavior inside Composite behavior is Composite subbehavior, then spin_cb() is used
    ## else spin().
    ## 
    ## @see Behavior::spin()
    ##
    def spin_cb(self):
        self.spin()
        states = []
        def function_process(name):
            save_stdout = sys.stdout
            sys.stdout = open('./debug/'+ str(name) + '.txt','w')
            try:
                print("here____")
                print 'Starting behaviour: {} at {}'.format(name,time.time())
                info = self._subbehavior_info[name]
                bhvr = info['behavior']
                print("Behavior",bhvr)

                # multi-robot behaviors always get spun
                # only spin single robot behaviors when they have a robot
                should_spin = True

                # try executing the subbehavior
                # if it throws an exception, catch it and pass it to the exception handler, which subclasses can override
                if should_spin:
                    try:
                        # print("Behavior name",bhvr)
                        if issubclass(bhvr.__class__, CompositeBehavior):
                            bhvr.spin_cb()
                        else:
                            bhvr.spin()
                    except:
                        exc = sys.exc_info()[0]
                        self.handle_subbehavior_exception(name, exc)
            finally:
                sys.stdout = save_stdout


        n_threads = len(self._subbehavior_info.keys())
        print 'Starting {} at {}'.format(n_threads, time.time())
        pool = ThreadPool(n_threads)
        pool.map(function_process, list(self._subbehavior_info.keys()))
        pool.close() 
        pool.join()

        print 'Completed at {}'.format(time.time())


    ## Override point for exception handling
    ## this is called whenever a subbehavior throws an exception during spin()
    ## subclasses of CompositeBehavior can override this to perform custom actions, such as removing the offending subbehavior
    ## the default implementation logs the exception and re-raises it
    def handle_subbehavior_exception(self, name, exception):
        # We only call this inside the above except
        #pylint: disable=misplaced-bare-raise
        logging.error("Exception occurred when spinning subbehavior named '" +
                      name + "': " + str(exception))
        traceback.print_exc()
        raise

    ## returns a tree of role_requirements
    def role_requirements(self):
        reqs = {}
        for name, info in self._subbehavior_info.items():
            r = info['behavior'].role_requirements()
            # r could be a RoleRequirements or a dict forming a subtree
            if isinstance(r, role_assignment.RoleRequirements):
                r.required = info['required']
                r.priority = info['priority']()
            # FIXME: should required and priority propogate if it's a tree?
            reqs[name] = r
        return reqs

    ## assignments is a tree with the same structure as that returned by role_requirements()
    ## the only difference is that leaf nodes are (RoleRequirements, OurRobot) tuples
    ## instead of just RoleRequirements
    def assign_roles(self, assignments):
        for name, subtree in assignments.items():
            self.subbehavior_with_name(name).assign_roles(subtree)

    def __str__(self) :
        desc = super().__str__()

        for name in self._subbehavior_info:
            bhvr = self._subbehavior_info[name]['behavior']
            # indent the subbehavior's description
            indent = '    '
            subdesc = str(bhvr)
            subdesc = "\n" + indent + re.sub(r'\n', '\n' + indent, subdesc)
            desc += subdesc

        return desc
