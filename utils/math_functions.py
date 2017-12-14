import math
from ctypes import *
from geometry import Vector2D
from config import *
def return_line_in_general_form(slope=None,intercept=None,point1=None,point2=None,r=None,theta=None):
  pass

def return_circle_in_general_form(center=None,radius=None,point1=None,point2=None,point3=None):
  pass

def line_circle_intersection(line,cirlce):
  pass

def point_in_a_triangle(point,triangle):
  pass

def line_intersection(line1,line2):
  pass

def line_ellipse_intersection(line,ellipse):
  pass

def normalize_angle(theta):
  if -math.pi < theta < math.pi:
    return theta
  if theta >= math.pi:
    return theta-2*math.pi
  if theta <= -math.pi:
    return theta + 2*math.pi

def getPointBehindTheBall(point ,theta):
  x = point.x -(1.2 * BOT_RADIUS) *(math.cos(theta))
  y = point.y -(1.2 * BOT_RADIUS) *(math.sin(theta))
  return Vector2D(int(x), int(y))

def deg_2_radian(theta):
  return theta * math.pi / 180.0

def dist(point1,point2):
  return math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2)

def angle_diff(point1,point2):
  return math.atan2(point2.y-point1.y,point2.x-point1.x)
