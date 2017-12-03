#To algin bot, ball and destination in a line
import skill_node
import math
import sys

sys.path.append('../../../skills_py/scripts/skills')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../plays_py/scripts/utils')

from config import *
from wrapperpy import *
from obstacle import Obstacle
from geometry import *

import skills_union
import sGoToBall
import sTurnToPoint

def execute(param, state, bot_id, pub):
	
