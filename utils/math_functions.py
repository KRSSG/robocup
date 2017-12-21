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


def angle_at_vextex(P1,P2,P3):
    a     = P2.dist(P3)
    b     = P1.dist(P3)
    c     = P1.dist(P2)
    value = (b**2 + c**2 - a**2)/(2*b*c)
    theta = math.acos(value)
    return math.atan2(math.tan(theta),1.0)

def area_of_triangle(p1,p2,p3):
    return math.abs(p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y))/2.0

def point_in_triangle(t, P):
    P1, P2, P3 = t[0], t[1], t[2]

    a  = area_of_triangle(P1,P2,P3)
    a1 = area_of_triangle(P,P2,P3)
    a2 = area_of_triangle(P1,P,P3)
    a3 = area_of_triangle(P1,P2,P)

    return a == a1+a2+a3
