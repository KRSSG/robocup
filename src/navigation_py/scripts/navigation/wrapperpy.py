from ctypes import *

# Create obstacle and Vector2D classes (in python)

from obstacle import *
import sys
sys.path.append('/home/vernwalrahul/rahul-krssg/src/plays_py/scripts/utils/')
from geometry import *

# Load shared library : navigation.so

NAV_LIB_DIR = "/home/vernwalrahul/rahul-krssg/devel/lib/"
NAV_LIB_NAME = "libnavigation.so"

class Vector_Obstacle(object):
	nav_lib = CDLL(NAV_LIB_DIR + NAV_LIB_NAME)
	nav_lib._vector_obstaclep_new.restype = c_void_p
	nav_lib._vector_obstaclep_new.argtypes = []
	nav_lib._vector_obstaclep_delete.restype = None
	nav_lib._vector_obstaclep_delete.argtypes = [c_void_p]
	nav_lib._vector_obstaclep_size.restype = c_int
	nav_lib._vector_obstaclep_size.argtypes = [c_void_p]
	nav_lib._vector_obstaclep_get.restype = POINTER(Obstacle)
	nav_lib._vector_obstaclep_get.argtypes = [c_void_p, c_int]
	nav_lib._vector_obstaclep_push_back.restype = None
	nav_lib._vector_obstaclep_push_back.argtypes = [c_void_p, POINTER(Obstacle)]

	def __init__(self):
		self.vector = Vector_Obstacle.nav_lib._vector_obstaclep_new()

	def __del__(self):
		Vector_Obstacle.nav_lib._vector_obstaclep_delete(self.vector)

	def __len__(self):
		return Vector_Obstacle.nav_lib._vector_obstaclep_size(self.vector)

	def __getitem__(self, i):
		if 0 <= i < len(self):
			return Vector_Obstacle.nav_lib._vector_obstaclep_get(self.vector, c_int(i))
		else:
			raise IndexError('Index out of range')

	def push_back(self, i):
		Vector_Obstacle.nav_lib._vector_obstaclep_push_back(self.vector, i)

	def __repr__(self):
		return '[{}]'.format(', '.join(str(self[i]) for i in range(len(self))))

class MergeSCurve(object):
	nav_lib = CDLL(NAV_LIB_DIR + NAV_LIB_NAME)
	nav_lib._MergeSCurvep_new.restype = c_void_p
	nav_lib._MergeSCurvep_new.argtypes = []
	nav_lib._MergeSCurvep_delete.restype = None
	nav_lib._MergeSCurvep_delete.argtypes = [c_void_p]
	nav_lib._MergeSCurvep_plan.restype = c_bool
	nav_lib._MergeSCurvep_plan.argtypes = [	c_void_p, POINTER(Vector2D), POINTER(Vector2D),
						POINTER(Vector2D), POINTER(Vector2D), c_void_p,
						c_int, c_int, c_bool				]
	def __init__(self):
		self.planner = MergeSCurve.nav_lib._MergeSCurvep_new()

	def __del__(self):
		MergeSCurve.nav_lib._MergeSCurvep_delete(self.planner)

	def plan(self, initial, final, pt1, pt2, obs, obstacle_count, current_id, teamBlue):
		return MergeSCurve.nav_lib._MergeSCurvep_plan(	self.planner,
								pointer(initial),
								pointer(final),
								pointer(pt1),
								pointer(pt2),
								obs.vector,
								c_int(obstacle_count),
								c_int(current_id),
								c_bool(teamBlue)	)
