from tactic import Tactic
import time
import sys
from math import *

sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
import obstacle
import sGoToPoint
import numpy as np
from numpy import array,inf

OPTIMAL_DISTANCE                        =  3.0*HALF_FIELD_MAXX/5
PASSING_PARAMETER                       =  5
OPTIMAL_DISTANCE_CONST                  =  10
BOT_ORIENT_CONST						=  0.2
OPP_DIST_CONST                          =  5



class TPosition(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParams = skills_union.SParam()


    def optimal_dist_wt(self, state,x,y,):
    	
     	AttackPos = Vector2D(int(state.homePos[attacker_bot_id].x), int(state.homePos[attacker_bot_id].y))
        if target_bot_id is attacker_bot_id:
            return 
        distance = AttackPos.dist(Vector2D(int(state.homePos[target_bot_id].x), int(Vector2D(state.homePos[target_bot_id].y))))
        normalizeDistance =  exp(-1*(OPTIMAL_DISTANCE - distance)/HALF_FIELD_MAXY)
        return normalizeDistance*OPTIMAL_DISTANCE_CONST

    def velo_direction_wt():
    	return 1

    def angle_diference_wt(self, state, attacker_bot_id, target_bot_id):
    	
    	theta = state.homePos[attacker_bot_id].theta
    	if target_bot_id is attacker_bot_id:
    		return

    	theta2 = state.homePos[target_bot_id].theta2

    	if fabs(theta2-theta)  > pi/2:
    		normalizeAngle = exp(-1*(fabs(theta2-theta)/pi))
    	else:
    		normalizeAngle = exp(fabs(theta2-theta)/pi)

    	return normalizeAngle*BOT_ORIENT_CONST

    def nearest_opp_distance_wt(self, state, attacker_bot_id, target_bot_id):
    	def line2point_dist(point,m,c):
    		return fabs((point.y - m*point.x)/ (m**2 + 1))
    	
    	m = (state.homePos[attacker_bot_id].y-state.homePos[target_bot_id].y)/ (state.homePos[attacker_bot_id].x-state.homePos[target_bot_id].x)
    	c =  state.homePos[attacker_bot_id].y - m*state.homePos[attacker_bot_id].x
    	min_dist = inf
    	for away_bot in xrange(len(state.awayPos)):
    		away_Pos = Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y))
    		dist = line2point_dist(away_Pos, m, c)
    		if dist < min_dist:
    			min_dist = dist
    	min_dist = exp(-1*min_dist/HALF_FIELD_MAXY)	
    	return min_dist*OPP_DIST_CONST  

    def gauss_dependencies(self, start, end):
        avg = 0.0

        for x in xrange(start, end):
            avg += x
        avg /= fabs(end-start+1)

        for x in xrange(start, end):
            std_dev += (x-avg)**2

        std_dev = sqrt(std_dev/ fabs(end-start+1))
        return avg, std_dev

    def gauss(self,x, mean, std_dev):   
        return (1/(sqrt(2*pi)*sqrt(std_dev))*exp(-0.5*((x-mean)/std_dev)**2))

    def scaled_lap_of_gauss(self,x, mean, std_dev):
        return (x**2-2(std_dev**2))/(std_dev**2)*exp(-0.5*(x/std_dev)**2)

    def calculate_bot_score(self, state, attacker_bot_id):
    	
    	for our_bot in xrange(len(state.homePos)):
    		scores = optimal_dist_wt*weights[0] + angle_diference_wt*weights[1] + nearest_opp_distance_wt*weights[2] + velo_direction_wt*weights[3]

    def getposxy(self, state, bot_id):
    	X = 1 + int(state.homePos[bot_id].x/ BOT_RADIUS)
    	Y = 1 + int(state.homePos[bot_id].y/ BOT_RADIUS)
    	return X, Y


    def assign_wt(self,state):
    	grid = np.zeros((3,3), dtype = float)
    	I,J = getposxy(state,bot_id)
    	grid[I][J]       = BOT_AREA_WT*(J+1-botPos.y+BOT_RADIUS)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
    	grid[I][J+1]     = BOT_AREA_WT*(botPos.y+BOT_RADIUS-J-1)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
    	grid[I-1][J+1]   = BOT_AREA_WT*(I-botPos.x+BOT_RADIUS)*(J+1-botPos.y+BOT_RADIUS)/ (BOT_RADIUS**2)
    	grid[I-1][J]     = BOT_AREA_WT*(botPos.x-BOT_RADIUS-I)*(botPos.y+BOT_RADIUS-J-1)/ (BOT_RADIUS**2)





    		

    def assign_wt(self, state):

    	self.grid = np.zeros((ROWS, COLS), dtype = float)
    	X_MEAN, X_DEV		     = self.gauss_dependencies(-HALF_FIELD_MAXX, HALF_FIELD_MAXX-SHIFT)
    	Y_MEAN, Y_DEV 			 = self.gauss_dependencies(HALF_FIELD_MINY,HALF_FIELD_MAXY)
    	X_GOAL_MEAN, X_GOAL_DEV  = self.gauss_dependencies(HALF_FIELD_MAXX-SHIFT, HALF_FIELD_MAXX)
    	SHIFT_COLS = SHIFT/(2*BOT_RADIUS)


	    our_bot_wt = 0.8
	    for our_bot in xrange(len(state.homePos)):
	    	botPos = Vector2D(int(state.homePos[our_bot]), int(state.homePos[our_bot]))
	    	I, J = self.getposxy(state, our_bot)
	    	factor[J][I]       = (J+1-botPos.y+BOT_RADIUS)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
	    	factor[J+1][I]   = (botPos.y+BOT_RADIUS-J-1)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
	    	factor[J+1][I-1]   = (I-botPos.x+BOT_RADIUS)*(J+1-botPos.y+BOT_RADIUS)/ (BOT_RADIUS**2)
	    	factor[J][I-1]     = (botPos.x-BOT_RADIUS-I)*(botPos.y+BOT_RADIUS-J-1)/ (BOT_RADIUS**2)






	    	grid[J][I]     += our_bot_wt*factor[J][I]
	    	grid[J+1][I] += our_bot_wt*factor[J+1][I+1]
	    	grid[J+1][I-1] += our_bot_wt*factor[J+1][I-1]
	    	grid[J][I-1]   += our_bot_wt*factor[J][I-1]



	    away_bot_wt = -0.8
	    for away_bot in xrange(len(state.away_Pos)):
	    	botPos = Vector2D(int(state.away_Pos[away_bot]), int(state.away_Pos[away_bot]))
	    	I, J = self.getposxy(state, away_bot)
	    	factor[J][I]       = (J+1-botPos.y+BOT_RADIUS)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
	    	factor[J+1][I+1]   = (botPos.y+BOT_RADIUS-J-1)*(botPos.x+BOT_RADIUS-I)/ (BOT_RADIUS**2)
	    	factor[J+1][I-1]   = (I-botPos.x+BOT_RADIUS)*(J+1-botPos.y+BOT_RADIUS)/ (BOT_RADIUS**2)
	    	factor[J][I-1]     = (botPos.x-BOT_RADIUS-I)*(botPos.y+BOT_RADIUS-J-1)/ (BOT_RADIUS**2)


	    	grid[J][I]     += away_bot_wt*factor[J][I]
	    	grid[J+1][I+1] += away_bot_wt*factor[J+1][I+1]
	    	grid[J+1][I-1] += away_bot_wt*factor[J+1][I-1]
	    	grid[J][I-1]   += away_bot_wt*factor[J][I-1]


    	
	    for X in xrange(-HALF_FIELD_MAXX, HALF_FIELD_MAXX, 2*BOT_RADIUS):
	    	for Y in xrange(HALF_FIELD_MINY, HALF_FIELD_MAXY, 2*BOT_RADIUS):
	    		if X < HALF_FIELD_MAXX-SHIFT:
	    			grid[Y][X] *= self.gauss(X, X_MEAN, X_DEV) + self.gauss(rows, Y_MEAN, Y_DEV)
	    		else:
	    			grid[Y][X] *= self.gauss(X, X_GOAL_MEAN, X_GOAL_DEV) + self.gauss(rows, Y_MEAN, Y_DEV)












    #def optimal_distance(state):
