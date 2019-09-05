from enum import Enum
import behavior
import random
import _GoToPoint_
import KickToPoint
import rospy
import time
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
import sys
import os
import rospy
from krssg_ssl_msgs.srv import path_plan
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.srv import bsServer
from velocity.profiler import *
from velocity.pid import pid
from velocity.pso import PSO
from velocity.error import Error
import numpy as np
from numpy import array,inf
import multiprocessing
import threading
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)



class Random1(behavior.Behavior):
	class State(Enum):
		setup = 1
		go=2

	def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):
			super(Random1,self).__init__()

			self.go1=Vector2D(HALF_FIELD_MAXX*0.2,HALF_FIELD_MAXY *0.9);
			self.go2=Vector2D(HALF_FIELD_MAXX*0.2,-HALF_FIELD_MAXY*0.9);
			self.target_point=self.go1
			self.bot_id = 0
			self.kubs=[]
			self.collector=None
			self.blocker=	None
			self.power=7.0
			self.go_clear = False
			self.safety_multiplier = 0.9
			self._block_object = None
			self.behavior_failed=False
			self._defend_goal_radius = 1.4
			self.block_line = None
			self.course_approch_thresh = course_approch_thresh

			self.add_state(Random1.State.setup,behavior.Behavior.State.running)
			self.add_state(Random1.State.go,behavior.Behavior.State.running)
			
			self.add_transition(behavior.Behavior.State.start,Random1.State.setup, lambda: True,"immediately")
			self.add_transition(Random1.State.setup,Random1.State.go, lambda: True,"set to go")


	def add_kub(self,kub):
		self.kub=kub

	def add_theta(self,theta):
		self.theta=theta

	def on_enter_setup(self):
		pass
	def execute_setup(self):
		pass
	def on_exit_setup(self):
		pass


	def get_point(self):
		if(self.target_point==self.go1):
			if(dist(self.kub.get_pos(),self.go1)<2*BOT_RADIUS):
				self.target_point=self.go2
			else:
				self.target_point=self.go1
		else:
			if(dist(self.kub.get_pos(),self.go2)<2*BOT_RADIUS):
				self.target_point=self.go1
			else:
				self.target_point=self.go2
		return self.target_point


	def on_enter_go(self):
		self.target_point=self.get_point()
		theta = normalize_angle(angle_diff(self.kub.get_pos(),self.target_point))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_go(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,False)
		for gf in generatingfunction:
			self.kub,target_point = gf
			self.target_point = self.get_point()
			if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
	def on_exit_go(self):
		pass




class Random2(behavior.Behavior):
	class State(Enum):
		setup = 1
		go = 2

	def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):
			super(Random2,self).__init__()

			self.go3=Vector2D(HALF_FIELD_MAXX*0.7,HALF_FIELD_MAXY *0.9);
			self.go4=Vector2D(HALF_FIELD_MAXX*0.7,-HALF_FIELD_MAXY*0.9);
			self.target_point=self.go3
			self.bot_id=1
			self.kubs=[]
			self.collector=None
			self.blocker=	None
			self.power=7.0
			self.go_clear = False
			self.safety_multiplier = 0.9
			self._block_object = None
			self.behavior_failed=False
			self._defend_goal_radius = 1.4
			self.block_line = None
			self.course_approch_thresh = course_approch_thresh

			self.add_state(Random2.State.setup,behavior.Behavior.State.running)
			self.add_state(Random2.State.go,behavior.Behavior.State.running)

			self.add_transition(behavior.Behavior.State.start,Random2.State.setup, lambda: True,"immediately")
			self.add_transition(Random2.State.setup,Random2.State.go, lambda: True,"set to go")
			

	def add_kub(self,kub):
		self.kub=kub
	
	def add_theta(self,theta):
		self.theta=theta

	def on_enter_setup(self):
		pass
	def execute_setup(self):
		pass
	def on_exit_setup(self):
		pass


	def get_point(self):
		if(self.target_point==self.go3):
			if(dist(self.kub.get_pos(),self.go3)<2*BOT_RADIUS):
				self.target_point=self.go4
			else:
				self.target_point=self.go3
		else:
			if(dist(self.kub.get_pos(),self.go4)<2*BOT_RADIUS):
				self.target_point=self.go3
			else:
				self.target_point=self.go4
		return self.target_point

	def on_enter_go(self):
		self.target_point=self.get_point()
		theta = normalize_angle(angle_diff(self.kub.get_pos(),self.target_point))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_go(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,False)
		for gf in generatingfunction:
			self.kub,target_point = gf
			self.target_point = self.get_point()
			if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
	def on_exit_go(self):
		pass


class Random3(behavior.Behavior):
	class State(Enum):
		setup = 1
		go = 2

	def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):
			super(Random3,self).__init__()

			self.go3=Vector2D(HALF_FIELD_MAXX,0)
			self.go4=Vector2D(0,0)
			self.target_point=self.go3
			self.bot_id=1
			self.kubs=[]
			self.collector=None
			self.blocker=	None
			self.power=7.0
			self.go_clear = False
			self.safety_multiplier = 0.9
			self._block_object = None
			self.behavior_failed=False
			self._defend_goal_radius = 1.4
			self.block_line = None
			self.course_approch_thresh = course_approch_thresh

			self.add_state(Random3.State.setup,behavior.Behavior.State.running)
			self.add_state(Random3.State.go,behavior.Behavior.State.running)

			self.add_transition(behavior.Behavior.State.start,Random3.State.setup, lambda: True,"immediately")
			self.add_transition(Random3.State.setup,Random3.State.go, lambda: True,"set to go")
			

	def add_kub(self,kub):
		self.kub=kub
	
	def add_theta(self,theta):
		self.theta=theta

	def on_enter_setup(self):
		pass
	def execute_setup(self):
		pass
	def on_exit_setup(self):
		pass


	def get_point(self):
		if(self.target_point==self.go3):
			if(dist(self.kub.get_pos(),self.go3)<2*BOT_RADIUS):
				self.target_point=self.go4
			else:
				self.target_point=self.go3
		else:
			if(dist(self.kub.get_pos(),self.go4)<2*BOT_RADIUS):
				self.target_point=self.go3
			else:
				self.target_point=self.go4
		return self.target_point

	def on_enter_go(self):
		self.target_point=self.get_point()
		theta = normalize_angle(angle_diff(self.kub.get_pos(),self.target_point))
		_GoToPoint_.init(self.kub, self.target_point, theta)
		pass

	def execute_go(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,False)
		for gf in generatingfunction:
			self.kub,target_point = gf
			self.target_point = self.get_point()
			if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
	def on_exit_go(self):
		pass
	





























	
	
	