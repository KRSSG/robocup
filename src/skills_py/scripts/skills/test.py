import skill_node
import math
import sys

sys.path.append('../../../navigation_py/scripts/navigation/')
sys.path.append('../../../plays_py/scripts/utils/')

from wrapperpy import MergeSCurve, Vector_Obstacle
from obstacle import Obstacle
from config import *
from geometry import Vector2D 

def execute(param,state,bot_id, pub):
	for i, bot in enumerate(state.homePos):
		