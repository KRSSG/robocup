# implement the helper functions similar to 'ssl_common/geometry.hpp'
import math
from ctypes import *

PI = 3.14159265358979323
INF = 9999999

class Vector2D(Structure):
	
	_fields_ =	[ ("x", c_int),
			 ("y", c_int) ]

	def __init__(self,x = None,y = None):
		if x is None:
			self.x = self.y = INF
		elif type(x) is Vector2D:
			self.x, self.y = x.x, x.y
		elif type(x) is int and type(y) is int:
			self.x, self.y = x, y
		else:
			raise Exception("Invalid constructor")

	def valid(self):
		if math.fabs(self.x) == INF or math.fabs(self.y) == INF :
			return False
		else :
			return True

	# Normalizes the angle (in radians) to be in the range (-pi, pi]
	def normalizeAngle(self,angle):
		if angle > PI :
			return angle - 2 * PI
		elif angle <= -PI :
			return angle + 2 * PI
		else :
			return angle

	# Sets the vector using polar coordinates
	def fromPolar(self,r,theta):
		v = Vector2D(0,0)
		v.x = r*math.cos(theta)
		v.y = r*math.sin(theta)
		return v

	# Returns the absolute value of the vector
	def abs(self,v):
		return math.sqrt(v.x*v.x+v.y*v.y)

	# Returns the squared absolute value of the vector
	def absSq(self,v):
		return (v.x*v.x+v.y*v.y)

	# Returns the angle made by the vector (head - self) in the range -pi to pi
	def angle(self,head):
		return math.atan2(self.y-head.y,self.x-head.x)

	# Returns the Eucledian distance between the 2 vectors
	def dist(self,another_point):
		return math.sqrt(math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2))

	# Returns the squared Eucledian distane between 2 vectors
	def distSq(self,another_point):
		return math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2)

	def dot(self,another_point):
		return self.x*another_point.x + self.y*another_point.y

	def __eq__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return True
		else :
			return False

	def __ne__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return False
		else :
			return True

	def __add__(self,another_point):
		return Vector2D(self.x+another_point.x,self.y+another_point.y)

	def __sub__(self,another_point):
		return Vector2D(self.x-another_point.x,self.y-another_point.y)

	def __mul__(self,scale):
		return Vector2D(self.x * scale , self.y * scale)

	def __truediv__(self,scale):
		if scale == 0 :
			raise Exception('Tried scaling down vector by zero')
		else :
			return Vector2D(self.x/scale,self.y/scale)
	
	# self is the point to be checked if it is within the circle with the center and radius as provided
	def intersects(self,center,radius):
		if self.distSq(center) < radius * radius :
			return True
		else :
			return False
	# self is the center of circle, checks whether the line made by the point1 and point2 intersects the circle
	def intersects(self,point1,point2,radius):
		# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
		d = point2 - point1
		f = point1 - self
		a = d.dot(d)
		b = 2*f.dot(d)
		c = f.dot(f) - radius*radius
		dis = b*b - 4*a*c
		if dis >=0 :
			dis = math.sqrt(dis)
			t1 = (dis-b)/(2*a)
			t2 = (-dis-b)/(2*a)
			if t1 >= 0 and t1 <= 1 or t2 >= 0 and t2 <=1 :
				return True
		return False


