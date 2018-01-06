##
## file 	error.py
## @brief      Error Information for PID
## 

class Error():
	def __init__(self):
		self.errorX = 0.0
		self.errorY = 0.0
		self.errorIX = 0.0
		self.errorIY = 0.0
		self.lastErrorX = 0.0
		self.lastErrorY = 0.0

		## @var errorX
		## Error in X
		## @var errorY
		## Error in y
		## @var errorIX
		## Integration of error in X
		## @var errorIY
		## Integration of error in Y
		## @var lastErrorX
		## last error in x
		## @var lastErrorY
		## last error in y
		##
		