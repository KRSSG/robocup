from ctypes import *

class Obstacle(Structure):
	_fields_ =	[ ("x",		c_float), 
			  ("y",		c_float),
			  ("x2",	c_float),
			  ("y2",	c_float), 
			  ("radius",	c_float) ]