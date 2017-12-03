from tactic import Tactic
import time
import sys

sys.path.append('../../..//skills_py/scripts/skills')
sys.path.append('../../..//plays_py/scripts/utils/')
sys.path.insert(0, '../../..//navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../..//navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
import obstacle
import sGoToPoint

from tactic import Tactic
from geometry import Vector2D
import skills_union
import sKick
import sKickToPoint
from math import *
from numpy import inf
from config import *
MAX_DRIBBLE_RANGE = 3
KICK_RANGE_THRESH = 3 * MAX_DRIBBLE_RANGE
THRES = 0.8
THETA_THRESH = 0.005
SHIFT = (HALF_FIELD_MAXX/9.0)

class TPosition(Tactic):
	def __init__(self,botID,state,params=None):
		self.botID = botID
		self.ballAim = Vector2D()
		self.goalieTarget = Vector2D(0,0)
		self.ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
		self.botPos = Vector2D(int(state.homePos[botID].x), int(state.homePos[botID].y))
		self.ballVel = Vector2D(int(state.ballVel.x) , int(state.ballVel.y))
		self.sParams = skills_union.SParam()

	def execute(self,state , pub):
		dist = self.ballPos.dist(self.botPos)
		'''		
		if(sqrt(self.ballVel.absSq()) < 0.02*MAX_BOT_SPEED and  fabs(state.ballPos.y) < OUR_GOAL_MAXY and state.ballPos.x > (HALF_FIELD_MAXX- (2 * DBOX_WIDTH) (HALF_FIELD_MAXX/5.0)):
		'''
		if fabs(state.ballPos.x) > fabs(HALF_FIELD_MAXX*4/5 ) and fabs(state.ballPos.y) < fabs(OUR_GOAL_MAXY*1.1) :
			self.sParams.GoToPointP.x = state.ballPos.x
		    self.sParams.GoToPointP.y = state.ballPos.y
		    self.sParams.GoToPointP.finalVelocity = 0
		    self.sParams.GoToPointP.finalslope = atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))
		    sGoToPoint.execute(self.sParams, state, self.botID, pub)
		    


		if(sqrt(self.ballVel.absSq(self.ballVel)) < 0.02*MAX_BOT_SPEED and fabs(state.ballPos.y) < OUR_GOAL_MAXY*1.1 and fabs(state.ballPos.x) > HALF_FIELD_MAXX -2 * DBOX_WIDTH- HALF_FIELD_MAXX/5.0):
		   if (dist >= DRIBBLER_BALL_THRESH):
			   self.sParams.GoToPointP.x = state.ballPos.x
			   self.sParams.GoToPointP.y = state.ballPos.y
			   self.sParams.GoToPointP.finalVelocity = 0
			   self.sParams.GoToPointP.finalslope = atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))
			   sGoToPoint.execute(self.sParams, state, self.botID, pub)
			   return
		   else:
			   self.sParams.KickP.power = 7.0
			   sKick.execute(self.sParams, state , self.botID, pub)
			   
		default_x = HALF_FIELD_MAXX + SHIFT
		
		if(state.ballPos.x >  (HALF_FIELD_MAXX - SHIFT) and state.ballPos.x < HALF_FIELD_MAXX):
			self.goalieTarget.x = int(HALF_FIELD_MAXX -SHIFT)
		else:
			self.goalieTarget.x  = int(default_x)
		
		striker = -1
		striker_dist = inf


		for oppID in xrange(5):
			if oppID == self.botID :
				continue
			oppPos = Vector2D(int(state.homePos[oppID].x), int(state.homePos[oppID].y))
			kick_range_test = sqrt(oppPos.absSq(oppPos-self.ballPos))

			if(kick_range_test < KICK_RANGE_THRESH and kick_range_test < striker_dist):
					striker = oppID
					striker_dist = kick_range_test
		if(striker is not -1):
			self.goalieTarget.y = int( ((state.ballPos.y - state.homePos[striker].y) / (state.ballPos.x - state.homePos[striker].x)) * (goalieTarget.x - state.ballPos.x) ) + state.ballPos.y
		else :
			if(state.ballVel.x == 0):
				self.goalieTarget.y = int(state.ballPos.y)
			else:
				if state.ballVel.x > 0:
					self.goalieTarget.y = (( state.ballVel.y / state.ballVel.x ) * ( self.goalieTarget.x \
					- state.ballPos.x ) ) + state.ballPos.y
				else:
					self.goalieTarget.y = 0
		if self.goalieTarget.y < OUR_GOAL_MINY/1.2 :
			self.goalieTarget.y = int(OUR_GOAL_MINY/1.2)
		elif self.goalieTarget.y > OUR_GOAL_MAXY/1.2 :
			self.goalieTarget.y = int(OUR_GOAL_MAXY/1.2)

		self.sParams.GoToPointP.x = self.goalieTarget.x
		self.sParams.GoToPointP.y = self.goalieTarget.y
		self.sParams.GoToPointP.finalVelocity = 0
		self.sParams.GoToPointP.finalslope = atan((state.ballPos.y - state.homePos[self.botID].y) / (state.ballPos.x - state.homePos[self.botID].x))
		sGoToPoint.execute(self.sParams, state, self.botID, pub)

	def isComplete(self, state):
		return False

	def updateParams(self, state):
		pass
