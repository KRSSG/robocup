##
## @brief      Velocity Profiling.
## 
## Return Velocity at current time according to velocity profiler
## on calling getVelocity
## @see getVelocity()
##
from math import *
class Velocity():
	# TODO
	# Start speed, final speed, maxacc
	
	##
	## @brief      Constructor of Velocity Profiling.
	##
	## @param      path       Path Points
	## @param      startTime  Starting time of profiling
	## @param[in]   currPosition 	Current Position of Kub
	## 
	##
	def __init__(self, path, startTime, currPosition):
		self.startTime = startTime
		self.path = path
		self.currPosition = currPosition
		self.distance_traversed = 0
		self.velocity = 0
		self.pathLength = self.GetPathLength()
		self.maxSpeed = 
		self.maxAcc = 
		self.startSpeed = 
		self.finalSpeed = 

	##
  	## @var startTime
  	## Starting time of velocity profiling
  	## @var path
	## points on path
	## @var currPosition
	## position of kub
	## @var distance_traversed
	## Total distance traversed by bot on path
	## @var velocity
	## Velocity according to profiling
	## @var pathLength
	## Total length of path
	## @var startSpeed
	## Initial Speed
	## @var finalSpeed
	## Final Speed



	##
	## @brief      Sends a stop.
	##
	## @return     velX, velY, errorX, errorY
	##
	def sendStop():
		return 0,0,0,0

	##
	## @brief      Sends a velocity.
	##
	## @param      velocity  Magnitude of velocity
	## @param      angle     angle of motion
	## @param      index     index of path list
	##
	## @return     velx, vely, errorx, errory
	##
	def sendVelocity(velocity, angle, index):
		velocity /= 1000
		velX = velocity*cos(angle)
		velY = velocity*sin(angle)
		errorX = self.path[index] - self.currPosition.x
		errorY = self.path[index] - self.currPosition.y

		return velX, velY, errorX, errorY

	##
	## @brief      Get index of current position of kub in path list.
	##
	def GetExpectedPositionIndex():
		distance = 0
		for i in xrange(1,len(self.path)):
			distance += self.path[i].dist(self.path[i-1])
			if distance > self.distance_traversed:
				return i
		return -1

	##
	## @brief      Total length of path
	##
	def GetPathLength():
		length = 0
		for i in xrange(1,len(self.path)):
			length += self.path[i].dist(self.path[i-1])
		return length

	##
	## @brief      Time to travell "pathlength" distance on path
	##
	## @param      distance    Total distance of path
	## @param      pathLength  Path length traversed
	## @param      maxSpeed    maximum speed
	## @param      startSpeed  start speed
	## @param      finalSpeed  final speed
	##
	##
	## Cases:-
	## 1> Triangle --> Maximum possible speed is not achieved
	## 
	## 2> Trapezoidal ---> Maximum possible speed attained for plateau
	##  time
	def getTime(pathLength):
		self.startSpeed = min(self.startSpeed,self.maxSpeed)
		self.finalSpeed = min(self.finalSpeed,self.maxSpeed)
		rampUpTime = (self.maxSpeed - self.startSpeed)/self.maxAcc
		# To be modified in function
		plateauTime = 0
		rampDownTime = -(self.finalSpeed - self.maxSpeed)/self.maxAcc
		rampUpDist = rampUpTime*(self.startSpeed + self.maxSpeed)/2.0
		# To be modified in function
		plateauDist = 0
		rampDownDist = rampDownTime*(self.maxSpeed + self.finalSpeed)/2.0
		if rampUpDist + rampUpDist > self.pathLength:
			# Triangle case :- Will Not attain maximum possible speed
			maxSpeed = sqrt(pow(self.finalSpeed,2) + pow(self.startSpeed,2) + 2*self.maxAcc*pathLength)/2.0
			rampUpTime = (maxSpeed - self.startSpeed)/self.maxAcc
			rampDownTime = -(self.finalSpeed - maxSpeed)/self.maxAcc
			rampUpDist = rampUpTime*(self.startSpeed + maxSpeed)/2.0
			rampDownTime = rampDownTime*(self.finalSpeed + maxSpeed)/2.0
			plateauTime = 0
			plateauDist = 0
		else:
			# Trapezoidal case
			# Attain Maximum Possible Speed for plateau Time
			plateauDist = self.pathLength - (rampUpDist + rampDownDist)
			plateauTime = plateauDist / self.maxSpeed

		if pathLength <= 0:
			return 0

		# Covered whole path
		if abs(pathLength - (rampUpDist + plateauDist + rampUpDist)) < 0.0001:
			return rampUpTime + plateauTime + rampDownTime

		if pathLength <= rampUpDist:
			# Time Calculations
			# 1/2*a*t^2 + t*vi -d = 0
			# t = -b + sqrt*(b^2 -4ac)/(2a)
			b = self.startSpeed
			a = self.maxAcc/2.0
			c = -pathLength
			root = sqrt(b*b - 4*a*c)
			try:
				alpha = (-b + root)/(2*a)
				beta = (-b - root)/(2*a)
			except:
				return "REPLAN"
			if alpha > 0 and alpha <= rampUpTime:
				return alpha
			else:
				return beta

		else if (pathLength <= rampUpDist + plateauDist ):
			position = pathLength - rampUpDist
			return rampUpTime + position/self.maxSpeed

		else if (pathLength < rampUpDist + plateauDist + rampDownDist):
			# Again Time Calculations
			position = pathLength - rampUpDist - plateauDist
			b = self.maxSpeed
			a = -self.maxAcc/2.0
			c = -position
			try:
				root = sqrt(b*b - 4*a*c)
			except:
				return "REPLAN"
			alpha = (-b + root)/(2*a)
			beta = (-b - root)/(2*a)
			if alpha > 0 and alpha < rampDownTime:
				return rampUpTime + plateauTime + alpha
			else:
				return rampUpTime + plateauTime + beta
		else:
			return rampUpTime + plateauTime + rampDownTime

	##
	## @brief      Check if Trapezoidal motion is possible
	##
	## @param      pathLength   Length of path
	## @param      maxSpeed     Maximum possible speed
	## @param      maxAcc       maximum possible accelaration
	## @param      timeIntoLap  currTime - startTime
	## @param      self.startSpeed   Starting Speed
	## @param      self.finalSpeed   Final Speed
	##
	def trapezoidalMotion(pathLength, maxSpeed, maxAcc, timeIntoLap,
						  startSpeed, finalSpeed):
		startSpeed = min(startSpeed,maxSpeed)
		finalSpeed = min(finalSpeed,maxSpeed)

		rampUpTime = (maxSpeed - startSpeed)/maxAcc
		# To be modified in function
		plateauTime = 0
		rampDownTime = -(finalSpeed - maxSpeed)/maxAcc
		rampUpDist = rampUpTime*(startSpeed + maxSpeed)/2.0
		# To be modified in function
		plateauDist = 0
		rampDownDist = rampDownTime*(maxSpeed + finalSpeed)/2.0

		if (rampUpDist + rampUpDist > pathLength):
			# Triangle Case, will not attain maxm possible speed
			maxSpeed = sqrt(pow(startSpeed,2) + pow(finalSpeed,2) + 2*maxAcc*pathLength)/2.0
			rampUpTime = (maxSpeed - startSpeed)/maxAcc
			plateauTime = 0
			rampDownTime = -(finalSpeed - maxSpeed)/maxAcc
			rampUpDist = rampUpTime*(startSpeed + maxSpeed)/2.0
			plateauDist = 0
			rampDownDist = rampDownTime*(finalSpeed + maxSpeed)/2.0
		else:
			plateauDist = pathLength - (rampUpDist + rampDownDist)
			plateauTime = plateauDist/maxSpeed
		if (timeIntoLap < 0):
			# Not started on path
			self.distance_traversed = 0
			self.velocity = startSpeed
			return False
		else if (timeIntoLap < rampUpTime):
			#
			# Accelerating at @maxAcc
			#
			 
			self.distance_traversed = startSpeed*timeIntoLap + 0.5*maxAcc*timeIntoLap*timeIntoLap
			self.velocity = startSpeed + maxAcc*timeIntoLap
			return True
  		else if(timeIntoLap < rampUpTime + plateauTime):
  			#
  			# Going at @maxSpeed
  			#
  			self.distance_traversed = rampUpDist + (timeIntoLap - rampUpTime)*maxSpeed
  			self.velocity = maxSpeed
  			return True
  		else if (timeIntoLap < rampUpTime + plateauTime + rampDownTime):
  			#
  			# on ramp down, deaccelarating at @maxAcc
  			#
  			timeIntoRampDown = timeIntoLap - (rampUpTime + plateauTime)
  			self.distance_traversed = 0.5*(-maxAcc) *timeIntoRampDown*timeIntoRampDown
  			self.distance_traversed += maxSpeed*timeIntoRampDown + (rampUpDist + plateauDist)
  			self.velocity = maxSpeed - maxSpeed*timeIntoRampDown
  			return True
  		else:
  			#
  			# At the end of path
  			#
  			self.distance_traversed = pathLength
  			self.velocity = finalSpeed
  			return False

  	##
  	## @brief      Gets the velocity.
  	##
  	def getVelocity():
  		return self.velocity

  	##
  	## @brief      Check if trapezoidal motion is possible
  	##
  	## @param      timeIntoLap  Currtime - startTime
  	##
  	def trapezoid(timeIntoLap):
  		valid = self.trapezoidalMotion(self.pathLength, self.maxSpeed, self.maxAcc, timeIntoLap, self.startSpeed, self.finalSpeed)
  		return valid

  	
  	