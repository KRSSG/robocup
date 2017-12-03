from tactic import Tactic
import time
import sys

sys.path.append('../../../skills_py/scripts/skills')
import sStop

class TStop(Tactic):

	def __init__(self, bot_id, state, param=None):
		super(TStop, self).__init__( bot_id, state, param)
		self.sParam = skills_union.SParam()
		
		# TODO: Need to set these threshold velocity values
		self.vel_x_threshold = 0.0
		self.vel_y_threshold = 0.0

	def execute(self, state, pub):
		sStop.execute(self.sParam, state, self.bot_id, pub)

	def isComplete(self, state):
		if state.homeVel[self.bot_id].x <= self.vel_x_threshold and state.homeVel[self.bot_id].y <= self.vel_y_threshold:
			return True
		elif time.time()-self.begin_time > self.time_out:
			return True
		else:
			return False

	def updateParams(self, state):
		# No parameter to update here
		pass
