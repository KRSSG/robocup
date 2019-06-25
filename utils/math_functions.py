print("importing math function")
import math
from ctypes import *
from geometry import Vector2D
from config import *


class Circle(object):

	def __init__(self, center=Vector2D(0, 0), radius=0):
		self.center = center
		self.radius = radius

	##
	# @brief      Check if circle intersects with given line
	# @param      line  The line
	##
	def if_intersect_with_line(self, line):
		if not isinstance(line, Line):
			raise ValueError("Expected instance of type Line, got %s" %type(line).__name__)
		distance = line.distance_from_point(self.center)
		return distance < self.radius

	##
	## @brief      Intersection point of circle and line
	## 
	## If circle and line doesn't intersect, it should raise an Error
	##
	## @param      line
	##
	def intersection_with_line(self, line):
		if not isinstance(line, Line):
			raise ValueError("Expected instance of type Line, got %s" %type(line).__name__)
		if not self.if_intersect_with_line(line):
			raise ValueError("Line and Circle doesn't intersect")
		C = self.center
		R = self.radius
		L = line
		theta = L.slope
		M = L.nearest_point_on_line(C)
		print(M.x,M.y)
		d = L.distance_from_point(C)
		r = math.sqrt(R**2 - d**2)
		print(r)
		# A = Vector2D(M.x + r * math.cos(theta), M.y + r * math.sin(theta))
		# B = Vector2D(M.x - r * math.cos(theta), M.y - r * math.sin(theta))
		A, B = Vector2D(), Vector2D()
		A.x = float(M.x + r * math.cos(theta))
		A.y = float(M.y + r * math.sin(theta))
		B.x = float(M.x - r * math.cos(theta))
		B.y = float(M.y - r * math.sin(theta))

		return A, B

	def if_point_in_circle(self, point):
		if not isinstance(point, Vector2D()):
			raise ValueError("Expected Vector2D() got %s" %type(point).__name__)
		distance = self.center.dist(point)
		return distance < self.radius


class Line(Structure):

	def __init__(self, point1=None, angle=None, point2=None):
		if not isinstance(point1, Vector2D):
			raise ValueError("point1 should be of type Vector2D, got %s" %type(point1).__name__)
		self.point = point1
		if angle is None:
			if not isinstance(point2, Vector2D):
				raise ValueError("point1 should be of type Vector2D, got %s" %type(point2).__name__)
			self.angle = math.atan2(point2.y - point1.y, point2.x - point1.x)
		else:
			self.angle = angle
		if self.angle > math.pi:
			self.angle = math.pi - self.angle
		elif self.angle < -math.pi:
			self.angle = math.pi + self.angle
		self.slope = math.tan(self.angle)
		   
	def if_intersect_with_circle(self, circle):
		if not isinstance(circle, Circle):
			raise ValueError("Expected instance of type Circle, got %s" %type(circle).__name__)
		return circle.if_intersect_with_line(self)

	def intersection_with_circle(self, circle):
		if not isinstance(circle, Circle):
			raise ValueError("Expected instance of type Circle, got %s" %type(circle).__name__)
		return circle.intersection_with_line(self)
	##
	# @brief      Intersection point of line from given line
	# @param      line2
	##


	# CHECK ---->  SLOPE
	def intersection_with_line(self, line2):
		if not isinstance(line, line2):
			raise ValueError("Expected Line instance, got %s" %type(line2).__name__)
		c1 = self.point.y - self.slope * self.point.x
		c2 = line2.point.y - line2.slope * line2.point.x

		m1 = self.slope
		m2 = line2.slope
		P = Vector2D()
		try:
			P.x = (c2 - c1) / (m1 - m2)
			P.y = (m1 * c2 - m2 * c1) / (m1 - m2)
			return P
		except:
			return None
	##
	# @brief      Find Prependicular distance of point from line
	# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	# 
	# @param       point
	##

	def distance_from_point(self, point):
		# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		# y = mx + c ---> mx - y + c = 0
		if not isinstance(point, Vector2D):
			raise ValueError("Expected Vector2D, got %s" %type(point).__name__)
		p = point
		a = self.slope
		b = -1
		c = self.point.y - self.slope * self.point.x

		distance = math.fabs(a * p.x + b * p.y + c) / math.sqrt(a**2 + b**2)
		return distance
	##
	# @brief      Find nearest point on line from given point
	# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	# 
	# @param       point
	##

	def projection_on_line(self, point, theta = math.pi/2):
		########################
		########### change fucntion to find projection on any angle
		########################
		if not isinstance(point, Vector2D):
			raise ValueError("Expected Vector2D, got %s" %type(point).__name__)
		p = point
		angle = math.atan(self.slope) - theta
		p = point
		a = self.slope
		b = -1
		c = self.point.y - self.slope * self.point.x
		closest_p = Vector2D()
		closest_p.x = float((b * (b * p.x - a * p.y) - a * c) / (a**2 + b**2))
		closest_p.y = float((a * (-b * p.x + a * p.y) - b * c) / (a**2 + b**2))
		return closest_p

	def angle_with_line(self,line):
		#######################################
		################### Testing remaining
		#######################################
		if not isinstance(line, Line):
			raise ValueError("Expected Line, got %s" %type(point).__name__)
		theta1 = atan(self.angle)
		theta2 = atan(line.angle)
		theta = math.fabs(theta1 - theta2)
		return min(theta, math.fabs(180 - theta))

	def normalized_vector(self):
		# angle = math.atan(self.slope)
		angle = self.angle
		return Vector2D(math.cos(angle), math.sin(angle))
	
	def nearest_point_on_line(self,point):
		t=(point.y-self.point.y)*math.sin(self.angle)+(point.x-self.point.x)*(math.cos(self.angle))
		x1=self.point.x+math.cos(self.angle)*t
		y1=self.point.y+math.sin(self.angle)*t
		point=Vector2D(x1,y1)
		return point

	##
	## @var slope  
	# Slope of line {tan(theta) = slope}
	## @var point  
	# Point on line
	## 


def point_in_a_triangle(point, triangle):
	pass


def line_ellipse_intersection(line, ellipse):
	pass


def normalize_angle(angle):
	if angle > math.pi:
		return angle - 2 * math.pi
	elif angle <= -math.pi:
		return angle + 2 * math.pi
	else:
		return angle



def magnitute(vector):
	return math.sqrt(vector.x * vector.x + vector.y * vector.y)

def line_circle_intersection(self,circle):
  pass


def direction(vector):
	return math.atan2(vector.y, vector.x)


def getPointBehindTheBall(point, theta, factor=3.5):
	x = point.x + (factor * BOT_RADIUS) * (math.cos(theta))
	y = point.y + (factor * BOT_RADIUS) * (math.sin(theta))
	return Vector2D(int(x), int(y))

def getPointToGo(point, theta):
	x = point.x + (1.5 * BOT_RADIUS) * (math.cos(theta))
	y = point.y + (1.5 * BOT_RADIUS) * (math.sin(theta))
	return Vector2D(int(x), int(y))

def deg_2_radian(theta):
	return theta * math.pi / 180.0


def radian_2_deg(theta):
	return theta * 180.0 / math.pi


def dist(point1, point2):
	return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def angle_diff(point1, point2):
	return math.atan2(point2.y - point1.y, point2.x - point1.x)

def stan_inverse(self,y,x):
	if atan2(y,x) < 1.5707963 and atan2(y,x) > -1.5707963:
		return atan2(y,x)
	if atan2(y,x) > 1.5707963 and atan2(y,x) < 3.14159265:
		return atan2(y,x)-3.14159265
	else:
		return atan2(y,x)+3.14159265

def vicinity_points(point1, point2, thresh=10):
	return dist(point1, point2) < thresh


def vicinity_theta(theta1, theta2, thresh=0.1):
	return abs(theta1 - theta2) < thresh


def angle_at_vextex(P1, P2, P3):
	a = P2.dist(P3)
	b = P1.dist(P3)
	c = P1.dist(P2)
	value = (b**2 + c**2 - a**2) / (2 * b * c)
	theta = math.acos(value)
	return math.atan2(math.tan(theta), 1.0)


def area_of_triangle(p1, p2, p3):
	return math.abs(p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2.0


def point_in_triangle(t, P):
	P1, P2, P3 = t[0], t[1], t[2]

	a = area_of_triangle(P1, P2, P3)
	a1 = area_of_triangle(P, P2, P3)
	a2 = area_of_triangle(P1, P, P3)
	a3 = area_of_triangle(P1, P2, P)

	return a == a1 + a2 + a3


def ball_in_front_of_bot(kub):
	theta1 = kub.get_pos().theta
	theta2 = math.atan2(kub.state.ballPos.y - kub.get_pos().y,
						kub.state.ballPos.x - kub.get_pos().x)
	return vicinity_theta(theta1, theta2, thresh=0.25) and vicinity_points(kub.get_pos(), kub.state.ballPos,
						thresh=BOT_RADIUS * 4)


def kub_has_ball(state, kub_id):
	theta1 = state.homePos[kub_id].theta
	theta2 = math.atan2(state.ballPos.y - state.homePos[kub_id].y,
						state.ballPos.x - state.homePos[kub_id].x)
	return vicinity_theta(theta1, theta2, thresh=0.25) and vicinity_points(state.homePos[kub_id],
						state.ballPos, thresh=BOT_RADIUS * 1.5)
