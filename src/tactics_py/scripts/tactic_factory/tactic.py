# Super class for all the tactics

import sys
import time

sys.path.insert(0,'./../../../plays_py/scripts')
sys.path.append('../../../plays_py/scripts')

from abc import ABCMeta, abstractmethod
from utils import tactics_union

class Tactic(object):
	# this default time is in milliseconds
	__metaclass__ = ABCMeta

	DEFAULT_TIMEOUT_PERIOD = 50  
	param = tactics_union.Param()
	def __init__(self, bot_id, state, param=None):
		
		# TO DO based on when param is None , use state to decide the param
		self.bot_id     = bot_id
		self.time_out   = Tactic.DEFAULT_TIMEOUT_PERIOD
		self.begin_time = time.time()
		self.param      = param
	
	@abstractmethod
	def execute(self,state):
		pass

	@abstractmethod
	def isComplete(self,state):
		pass

	@abstractmethod
	def updateParams(self,state):
		pass
