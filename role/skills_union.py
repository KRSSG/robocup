from ctypes import *

class SpinP(Structure):
	_fields_= [("radPerSec", c_float)]

class StopP(Structure):
	_fields_ = []

class GoToBallP(Structure):
	_fields_ = [("intercept", c_bool)]

class KickP(Structure):
	_fields_ = [("power", c_float)]

class KickToPointP(Structure):
	_fields_ = [("power", c_float), 
	("x", c_float), 
	("y", c_float)]

class VelocityP(Structure):
	_fields_ = [("v_x", c_float), 
	("v_y", c_float), 
	("v_t", c_float)]

class DefendPointP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalslope", c_float), 
	("radius", c_float)]

class DribbleToPointP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalslope", c_float), 
	("radius", c_float)]

class GoalKeepingP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalslope", c_float), 
	("radius", c_float)]

class TurnToAngleP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalslope", c_float), 
	("radius", c_float)]

class TurnToPointP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("max_omega", c_float)]

class DribbleTurnP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("max_omega", c_float), 
	("turn_radius", c_float)]

class MoveOnArcP(Structure):
	_fields_ = [("centrex", c_float), 
	("centrey", c_float), 
	("finalx", c_float), 
	("finaly", c_float)]	

class GoToPointP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalSlope", c_float), 
	("align", c_bool), 
	("finalVelocity", c_float)]

class MoveWithDribblerP(Structure):
	_fields_ = [("v_x", c_float), 
	("v_y", c_float), 
	("v_t", c_float)]

"""
Create union of all the above classes
"""
class SParam(Union):
	_fields_ = [("SpinP", SpinP),
	("StopP", StopP),
	("GoToBallP", GoToBallP),
	("KickP", KickP), 
	("KickToPointP", KickToPointP),
	("VelocityP", VelocityP),
	("DefendPointP", DefendPointP), 
	("DribbleToPointP", DribbleToPointP),
	("GoalKeepingP", GoalKeepingP),
	("TurnToAngleP", TurnToAngleP),
	("TurnToPointP", TurnToPointP),
	("DribbleTurnP", DribbleTurnP),
	("GoToPointP", GoToPointP),
	("MoveWithDribblerP", MoveWithDribblerP),
	("MoveOnArcP", MoveOnArcP)]
