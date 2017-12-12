# Super class for all the plays
import time
from utils import 

class Play(object):
	# this default time is in milliseconds
	DEFAULT_TIMEOUT_PERIOD = 50

	def __init__(self, name, time_out=DEFAULT_TIMEOUT_PERIOD):
		self.name       = name
		self.time_out   = time_out
		self.begin_time = int(round(time.time() * 1000))


	def timed_out(self):
		"""
		return True if time out
		"""
		if int(round(time.time() * 1000))-self.begin_time > self.time_out:
			return True
		else:
			return False

