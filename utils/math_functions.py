import math
from ctypes import *
from geometry import Vector2D
from config import *

class Circle:
  def __init__(self,center=Vector2D(0,0),,radius=0):
    self.center = center
    self.radius = radius

  def line_circle_intersection(self,line):
    pass


class Line:
  """docstring for Line"""
  def __init__(self, point1=None,slope=None,point2=None):
    self.point = point1
    if slope is None:
      self.slope = math.atan2(point2.y-point1.y,point2.x-point1.x)
    else:
      self.slope = slope

    def ne


  def line_circle_intersection(self,circle):
    pass

  def line_intersection(self,line2):
    pass

  def nearest_point(self,point):
    ##intersection of self with line(point,slope)
    slope = math.pi/2 + self.slope
    return point

    


def point_in_a_triangle(point,triangle):
  pass


def line_ellipse_intersection(line,ellipse):
  pass



def normalize_angle(angle):
  if angle > math.pi :
    return angle - 2 * math.pi
  elif angle <= -math.pi :
    return angle + 2 * math.pi
  else :
    return angle

def magnitute(vector):
  return math.sqrt(vector.x*vector.x+vector.y*vector.y)

def direction(vector):
  return math.atan2(vector.y,vector.x)

def getPointBehindTheBall(point ,theta):
  x = point.x -(3.5 * BOT_RADIUS) *(math.cos(theta))
  y = point.y -(3.5 * BOT_RADIUS) *(math.sin(theta))
  return Vector2D(int(x), int(y))

def deg_2_radian(theta):
  return theta * math.pi / 180.0

def radian_2_deg(theta):
  return theta*180.0 /math.pi 

def dist(point1,point2):
  return math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2)

def angle_diff(point1,point2):
  return math.atan2(point2.y-point1.y,point2.x-point1.x)

def vicinity_points(point1,point2,thresh=10):
  return  dist(point1,point2)<thresh

def vicinity_theta(theta1,theta2,thresh=0.1):
  return abs(theta1-theta2)<thresh

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

def ball_in_front_of_bot(kub):
  theta1 = kub.get_pos().theta
  theta2 = math.atan2(kub.state.ballPos.y-kub.get_pos().y,kub.state.ballPos.x-kub.get_pos().x)
  return vicinity_theta(theta1,theta2,thresh=0.25) and vicinity_points(kub.get_pos(),kub.state.ballPos,thresh=BOT_RADIUS*4)


def kub_has_ball(state,kub_id):
  theta1 = state.homePos[kub_id].theta
  theta2 = math.atan2(state.ballPos.y-state.homePos[kub_id].y,state.ballPos.x-state.homePos[kub_id].x)
  return vicinity_theta(theta1,theta2,thresh=0.25) and vicinity_points(state.homePos[kub_id],state.ballPos,thresh=BOT_RADIUS*1.5)
