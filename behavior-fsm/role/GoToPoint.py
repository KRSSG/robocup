from enum import Enum
import behavior
import config
import _GoToPoint
class GoToPoint(behavior.Behavior):
	"""docstring for GoToPoint"""
	##
	## @brief      Class for state.
	##
	class State(Enum):
		setup = 1 
		drive = 2

	##
	## @brief      Constructs the object.
	##
	## @param      self   The object
	## @param      point  The point
	##
	def __init__(self,kub,state,point):
		super(GoToPoint, self).__init__()
		self.kub = kub
		self.state = state

		self.target_point = point

		self.add_state(GoToPoint.State.setup,
			behaviour.Behaviour.State.running)
		self.add_state(GoToPoint.State.drive,
			behaviour.Behaviour.State.running)
		

		self.add_transition(behaviour.Behaviour.State.start,
			GoToPoint.State.setup,lambda: True,'immediately')

		self.add_transition(GoToPoint.State.setup,
			GoToPoint.State.drive,lambda: self.target_present,'setup')

		self.add_transition(GoToPoint.State.drive,
			behaviour.Behaviour.State.completed,lambda:self.at_new_point(),'complete')

		self.add_transition(behaviour.Behaviour.State.completed,
			behaviour.Behaviour.State.start,lambda:not self.at_new_point(),'complete')
	##
	## @brief      { function_description }
	##
	## @param      self  The object
	##
	## @return     { description_of_the_return_value }
	##
	def target_present(self):
		return self.target_point is not None

	##
	## @brief      { function_description }
	##
	## @return     { description_of_the_return_value }
	##
	def at_new_point():
		return self.new_point.dist(self.target_point) < config.DISTANCE_THRESHOLD

		
	def on_enter_setup(self):
		pass
	def execute_setup(self):
		_GoToPoint.init(self.kub,self.target_point)
		pass
		
	def on_exit_setup(self):
		pass

	def on_enter_drive(self):
		pass

	def terminate(self):
		super().terminate()
	##
	## @brief      { function_description }
	##
	## @param      self   The object
	## @param      kub    The kub
	## @param      state  The state
	##
	## @return     { description_of_the_return_value }
	##
	def execute_drive(self):
		_GoToPoint.run()
		##
		##
		##velo profilling data		
		# ## for each velo in velo_profling vector
		# kub.move(vx,vy)
		# kub.turn(vw)
		# kub.execute(state)
		##
		## 
		##
		self.new_point = kub.get_pos(self.state)
		

	
	def on_exit_drive(self):
		pass




