from ctypes import *

"""
-> define one separate class for each of the tactics parameters
   as given in 'tactics/tactics.h'
-> finally create an union of all the above classes
"""

class GoalieP(Structure):
	_fields_ = []

class ClearP(Structure):
	_fields_ = []

class BlockP(Structure):
	_fields_ = [("dist", c_float), 
	("side", c_int)]

class DefendLineP(Structure):
	_fields_ = [("x1", c_int), 
	("y1", c_int), 
	("x2", c_int), 
	("y2", c_int),
	("radius", c_int)]

class PositionP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalSlope", c_float), 
	("finalVelocity", c_float), 
	("align", c_bool)]

class PositionForStartP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalSlope", c_float), 
	("finalVelocity", c_float), 
	("align", c_bool)]

class PositionForPenaltyP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float), 
	("finalSlope", c_float), 
	("finalVelocity", c_float), 
	("align", c_bool)]

class StopP(Structure):
	_fields_ = []

class VelocityP(Structure):
	_fields_ = [("v_x", c_float), 
	("v_y", c_float), 
	("v_t", c_float)]

class KickP(Structure):
	_fields_ = [("power", c_float)]

class DefendPointP(Structure):
	_fields_ = [("x", c_int), 
	("y", c_int), 
	("radius", c_int)]

class MarkBotP(Structure):
	_fields_ = [("awayBotID", c_int)]

class AttackSupportP(Structure):
	_fields_ = [("id", c_int)]

class PassP(Structure):
	_fields_ = [("x", c_float), 
	("y", c_float)]

class DefendARc_left(Structure):
	_fields_ = []

class DefendARc_right(Structure):
	_fields_ = []

class IntercptP(Structure):
	_fields_ = [("awayBotID", c_int), 
	("where", c_int)]

class DribbleTurnPassP(Structure):
	_fields_ = [("x", c_int), 
	("y", c_int)]



"""
Create union of all the above classes
"""
class Param(Union):
	_fields_ = [("GoalieP", GoalieP), 
	("ClearP", ClearP), 
	("BlockP", BlockP), 
	("DefendLineP", DefendLineP),
	("PositionP", PositionP),
	("PositionForStartP", PositionForStartP),
	("PositionForPenaltyP", PositionForPenaltyP),
	("StopP", StopP), 
	("VelocityP", VelocityP),
	("KickP", KickP),
	("DefendPointP", DefendPointP),
	("MarkBotP", MarkBotP),
	("AttackSupportP", AttackSupportP),
	("PassP", PassP),
	("DefendARc_left", DefendARc_left), 
	("DefendARc_right", DefendARc_right),
	("IntercptP", IntercptP), 
	("DribbleTurnPassP", DribbleTurnPassP)]
