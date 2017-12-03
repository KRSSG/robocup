from tactic import Tactic
import time
import sys

sys.path.append('/home/mehul/Documents/KRSSG/src/skills_py/scripts/skills')
sys.path.append('/home/mehul/Documents/KRSSG/src/plays_py/scripts/utils/')
from geometry import Vector2D 
import skills_union
import sGoToPoint
#import sKick

class TPosition(Tactic):

	def __init__(self, bot_id, state, param=None):
		super(TPosition, self).__init__( bot_id, state, param)
		self.sParam = skills_union.SParam()

		self.destination = Vector2D(int(self.param.PositionP.x), int(self.param.PositionP.y))
		self.threshold = 20.0

	def execute(self, state, pub):
		
		self.sParam.GoToPointP.x             = self.param.PositionP.x
		self.sParam.GoToPointP.y             = self.param.PositionP.y
		self.sParam.GoToPointP.align         = self.param.PositionP.align = False
		self.sParam.GoToPointP.finalslope    = self.param.PositionP.finalSlope = 0
		self.sParam.GoToPointP.finalVelocity = self.param.PositionP.finalVelocity = 0
		self.sParam.KickP.power=1

		sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
		#sKick.execute(self.sParam, state, self.bot_id, pub)

	def isComplete(self, state):
		# TO DO use threshold distance instead of actual co ordinates
		if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
			return True
		elif time.time()-self.begin_time > self.time_out:
			return True
		else:
			return False

	def updateParams(self, state):
		# No parameter to update here
		pass
