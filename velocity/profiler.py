##
## @brief      Velocity Profiling.
## 
## Return Velocity at current time according to velocity profiler
## on calling getVelocity
## @see getVelocity()
##
import sys

from math import *
from utils.geometry import Vector2D
from utils.config import *
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
		self.maxSpeed = MAX_BOT_SPEED
		self.maxAcc = MAX_BOT_ACCELERATION
		self.startSpeed = 1000
		self.finalSpeed = 0
		self.motionAngle = []
		self.majorAxis = 10.0
  	  	self.minorAxis = 2.0

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
	## @var motionAngle
	## Angles at each position
	## @var majorAxis
	## factor in ellipse
	## @var minorAxis
	## factor in ellipse



	##
	## @brief      Sends a stop.
	##
	## @return     velX, velY, errorX, errorY
	##
	def sendStop(self):
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
	def sendVelocity(self,velocity, angle, index):
		velX = velocity*cos(angle)
		velY = velocity*sin(angle)
		# print(velX,velY,angle,index,self.motionAngle[index])
		errorX = self.path[index].x - self.currPosition.x
		errorY = self.path[index].y - self.currPosition.y
		# print(velX,velY,errorX,errorY)
		return velX, velY, errorX, errorY
	##
	## @brief      Get index of current position of kub in path list.
	##
	def GetExpectedPositionIndex(self):
		distance = 0
		for i in xrange(1,len(self.path)):
			distance += self.path[i].dist(self.path[i-1])
			if distance > self.distance_traversed:
				return i
		return -1

	##
	## @brief      Total length of path
	##
	def GetPathLength(self):
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
	def getTime(self,pathLength):
		self.startSpeed = min(self.startSpeed,self.maxSpeed)
		self.finalSpeed = min(self.finalSpeed,self.maxSpeed)
		self.rampUpTime = (self.maxSpeed - self.startSpeed)/self.maxAcc
		# To be modified in function
		self.plateauTime = 0
		self.rampDownTime = -(self.finalSpeed - self.maxSpeed)/self.maxAcc
		self.rampUpDist = self.rampUpTime*(self.startSpeed + self.maxSpeed)/2.0
		# To be modified in function
		self.plateauDist = 0
		self.rampDownDist = self.rampDownTime*(self.maxSpeed + self.finalSpeed)/2.0
		if self.rampUpDist + self.rampDownDist > self.pathLength:
			# Triangle case :- Will Not attain maximum possible speed
			self.maxSpeed = sqrt((pow(self.finalSpeed,2) + pow(self.startSpeed,2) + 2*self.maxAcc*pathLength)/2.0)
			self.rampUpTime = (self.maxSpeed - self.startSpeed)/self.maxAcc
			self.rampDownTime = -(self.finalSpeed - self.maxSpeed)/self.maxAcc
			self.rampUpDist = self.rampUpTime*(self.startSpeed + self.maxSpeed)/2.0
			self.rampDownDist = self.rampDownTime*(self.finalSpeed + self.maxSpeed)/2.0
			self.plateauTime = 0
			self.plateauDist = 0
		else:
			# Trapezoidal case
			# Attain Maximum Possible Speed for plateau Time
			self.plateauDist = self.pathLength - (self.rampUpDist + self.rampDownDist)
			self.plateauTime = self.plateauDist / self.maxSpeed

		if pathLength <= 0:
			return 0

		# Covered whole path
		if abs(pathLength - (self.rampUpDist + self.plateauDist + self.rampUpDist)) < 0.0001:
			return self.rampUpTime + self.plateauTime + self.rampDownTime

		if pathLength <= self.rampUpDist:
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
			if alpha > 0 and alpha <= self.rampUpTime:
				return alpha
			else:
				return beta

		elif (pathLength <= self.rampUpDist + self.plateauDist ):
			position = pathLength - self.rampUpDist
			return self.rampUpTime + position/self.maxSpeed

		elif (pathLength < self.rampUpDist + self.plateauDist + self.rampDownDist):
			# Again Time Calculations
			position = pathLength - self.rampUpDist - self.plateauDist
			b = self.maxSpeed
			a = -self.maxAcc/2.0
			c = -position
			try:
				root = sqrt(b*b - 4*a*c)
			except:
				return "REPLAN"
			alpha = (-b + root)/(2*a)
			beta = (-b - root)/(2*a)
			if alpha > 0 and alpha < self.rampDownTime:
				return self.rampUpTime + self.plateauTime + alpha
			else:
				return self.rampUpTime + self.plateauTime + beta
		else:
			return self.rampUpTime + self.plateauTime + self.rampDownTime

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
	def trapezoidalMotion(self,pathLength, maxSpeed, maxAcc, timeIntoLap,
						  startSpeed, finalSpeed):
		startSpeed = min(startSpeed,maxSpeed)
		finalSpeed = min(finalSpeed,maxSpeed)

		# self.rampUpTime = (maxSpeed - startSpeed)/maxAcc
		# # To be modified in function
		# self.plateauTime = 0
		# self.rampDownTime = -(finalSpeed - maxSpeed)/maxAcc
		# self.rampUpDist = self.rampUpTime*(startSpeed + maxSpeed)/2.0
		# # To be modified in function
		# self.plateauDist = 0
		# self.rampDownDist = self.rampDownTime*(maxSpeed + finalSpeed)/2.0
		# # print("Time in lap",timeIntoLap)
		# if (self.rampUpDist + self.rampUpDist > pathLength):
		# 	# Triangle Case, will not attain maxm possible speed
		# 	maxSpeed = sqrt(pow(startSpeed,2) + pow(finalSpeed,2) + 2*maxAcc*pathLength)/2.0
		# 	self.rampUpTime = (maxSpeed - startSpeed)/maxAcc
		# 	self.plateauTime = 0
		# 	self.rampDownTime = -(finalSpeed - maxSpeed)/maxAcc
		# 	self.rampUpDist = self.rampUpTime*(startSpeed + maxSpeed)/2.0
		# 	self.plateauDist = 0
		# 	self.rampDownDist = self.rampDownTime*(finalSpeed + maxSpeed)/2.0
		# else:
		# 	self.plateauDist = pathLength - (self.rampUpDist + self.rampDownDist)
		# 	self.plateauTime = self.plateauDist/maxSpeed
		if (timeIntoLap < 0):
			# Not started on path
			print("Not started on path")
			self.distance_traversed = 0
			self.velocity = startSpeed
			return False
		elif (timeIntoLap < self.rampUpTime):
			#
			# Accelerating at @maxAcc
			#
			 
			self.distance_traversed = startSpeed*timeIntoLap + 0.5*maxAcc*timeIntoLap*timeIntoLap
			self.velocity = startSpeed + maxAcc*timeIntoLap
			return True
  		elif(timeIntoLap < self.rampUpTime + self.plateauTime):
  			#
  			# Going at @maxSpeed
  			#
  			self.distance_traversed = self.rampUpDist + (timeIntoLap - self.rampUpTime)*maxSpeed
  			self.velocity = maxSpeed
  			return True
  		elif (timeIntoLap < self.rampUpTime + self.plateauTime + self.rampDownTime):
  			#
  			# on ramp down, deaccelarating at @maxAcc
  			#
  			timeIntoRampDown = timeIntoLap - (self.rampUpTime + self.plateauTime)
  			self.distance_traversed = 0.5*(-maxAcc) *timeIntoRampDown*timeIntoRampDown
  			self.distance_traversed += maxSpeed*timeIntoRampDown + (self.rampUpDist + self.plateauDist)
  			self.velocity = maxSpeed - maxAcc*timeIntoRampDown
  			return True
  		else:
  			#
  			# At the end of path
  			#
  			print("At the end of path")
  			print(timeIntoLap,self.rampUpTime + self.plateauTime + self.rampDownTime)
  			self.distance_traversed = pathLength
  			self.velocity = finalSpeed
  			return False

  	##
  	## @brief      Gets the velocity.
  	##
  	def getVelocity(self):
  		return self.velocity

  	##
  	## @brief      Check if trapezoidal motion is possible
  	##
  	## @param      timeIntoLap  Currtime - startTime
  	##
  	def trapezoid(self,timeIntoLap,pos):
  		self.currPosition = pos
  		valid = self.trapezoidalMotion(self.pathLength, self.maxSpeed, self.maxAcc, timeIntoLap, self.startSpeed, self.finalSpeed)
  		return valid

  	def updateAngle(self):
  		for i in xrange(0,len(self.path)):
  			if i == 0:
  				dx = self.path[i+1].x - self.path[i].x
  				dy = self.path[i+1].y - self.path[i].y
  				self.motionAngle = self.motionAngle + [atan2(dy,dx)]
  			elif i == len(self.path) - 1:
  				dx = self.path[i].x - self.path[i-1].x
  				dy = self.path[i].y - self.path[i-1].y
  				self.motionAngle = self.motionAngle + [atan2(dy,dx)]
  			else:
  				dx = self.path[i+1].x - self.path[i-1].x
  				dy = self.path[i+1].y - self.path[i-1].y
  				self.motionAngle = self.motionAngle + [atan2(dy,dx)] 	
  			

  	def ellipse(self, myPos, oppPos, angle):
  		majorAxis = 10.0
  		minorAxis = 2.0
  		a = 1.0*majorAxis*BOT_RADIUS/2.0
  		b = 1.0*minorAxis*BOT_RADIUS/2.0
  		xOne = myPos.x + a*cos(angle)
  		yOne = myPos.y + a*sin(angle)
  		xTwo = oppPos.x
  		yTwo = oppPos.y
  		vOne = (cos(angle)*(xTwo - xOne) + sin(angle)*(yTwo - yOne))/a
  		vTwo = (sin(angle)*(xTwo - xOne) - cos(angle)*(yTwo - yOne))/b
  		value = pow(vOne,2) + pow(vTwo,2)
  		if value <= 1:
  			return 1
  		else:
  			return 0
  	
  	