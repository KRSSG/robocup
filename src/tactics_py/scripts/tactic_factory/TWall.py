from tactic import Tactic
import time
import sys

sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')

from geometry import * 
import skills_union
from config import *
import obstacle
import sGoToPoint
import sStop
from tactic import Tactic
from geometry import Vector2D
import skills_union
import sKickToPoint
import sGoToPoint
from math import *
from numpy import inf,array,linalg
from config import *

DIST_THRESH=BOT_RADIUS
GOAL_CENTER=Vector2D(int(-HALF_FIELD_MAXX), int(0))
ARC_RADIUS=DBOX_WIDTH*1.2
class TWall(Tactic):
    def __init__(self,bot_id,state,params=None, wall=None):
        super(TWall, self).__init__( bot_id, state, params)
        self.bot_id      =    bot_id
        self.wall_bots = wall
        self.sParam    =    skills_union.SParam()
        self.iState     =    "POSITION"
        


    def  isComplete(self,state,tParams):
        return iState == "KICK"

    def isActiveTactic():
        return iState is not "KICK"

    def updateParams(self, state):
        pass
   
   
    def execute(self, state , pub):
        #print("called me ",self.bot_id)
        #print(self.wall_bots)
        def inside_DBOX(botPos):
            #print("checking if ID")
            if(botPos.dist(GOAL_CENTER)<ARC_RADIUS):
                return 1
            return 0
            
        def ball_in_danger_zone(ballPos):
            #print("checking if BIDZ")
            if(ballPos.dist(GOAL_CENTER)<2*ARC_RADIUS):
                return 1
            return 0
                        
        def find_point_to_kick():
            kickingPoint=Vector2D(int(HALF_FIELD_MAXX), int(0))  
            return kickingPoint

        other_bot_id=None
        
        if(self.wall_bots[0]==self.bot_id):
            other_bot_id=self.wall_bots[1]
        else:
            other_bot_id=self.wall_bots[0]  

        print("\n\n\n HECK "+str(other_bot_id)+"Wall"+str(self.wall_bots)+"\n\n\n\n")
        ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        botPos =Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        otherbotPos=Vector2D(int(state.homePos[other_bot_id].x), int(state.homePos[other_bot_id].y))
        ball_angle_wrt_goalC=ballPos.angle(GOAL_CENTER)
        my_angle_wrt_goalC=botPos.angle(GOAL_CENTER)
        other_angle_wrt_goalC=otherbotPos.angle(GOAL_CENTER)

        #print (" D_________")
        if(inside_DBOX(botPos)):
            # print("INSIDE_DBOX")
            self.sParam.GoToPointP.x=GOAL_CENTER.x+ 1.2*ARC_RADIUS*cos(my_angle_wrt_goalC)
            self.sParam.GoToPointP.y=GOAL_CENTER.y+ 1.2*ARC_RADIUS*sin(my_angle_wrt_goalC)
            self.sParam.finalslope=ballPos.angle(botPos)
            # print("me ",self.bot_id,"going to point", self.sParam.GoToPointP.x, self.sParam.GoToPointP.y)
            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return

        if(ball_in_danger_zone(ballPos)):
            if(ballPos.dist(botPos)<ballPos.dist(otherbotPos)):
                kickingPoint=find_point_to_kick()
                self.sParam.KickToPointP.x=kickingPoint.x
                self.sParam.KickToPointP.y=kickingPoint.y
                self.sParam.KickToPointP.power=7
                # print("me ",self.bot_id,"kicking to point ",kickingPoint.x,kickingPoint.y)
                sKickToPoint.execute(self.sParam, state, self.bot_id, pub)
                return
        
        # print(" calculating possible angle")
        possible_angle_1=ball_angle_wrt_goalC-atan(1.0*BOT_RADIUS/botPos.dist(GOAL_CENTER))
        possible_angle_2=ball_angle_wrt_goalC+atan(1.0*BOT_RADIUS/botPos.dist(GOAL_CENTER))    

        if(my_angle_wrt_goalC<other_angle_wrt_goalC):
            # print("me1other2=")
            # print("me ",self.bot_id,"at angle",my_angle_wrt_goalC*180/pi)
            # print("other ",other_bot_id,"at",other_angle_wrt_goalC*180/pi)
            self.sParam.GoToPointP.x=GOAL_CENTER.x+ ARC_RADIUS*cos(possible_angle_1)
            self.sParam.GoToPointP.y=GOAL_CENTER.y+ ARC_RADIUS*sin(possible_angle_1)
            self.sParam.finalslope=ballPos.angle(botPos)
            # print("me ",self.bot_id,"going to point", self.sParam.GoToPointP.x, self.sParam.GoToPointP.y)
            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return
        else:
            # print("me2other1")
            self.sParam.GoToPointP.x=GOAL_CENTER.x+ ARC_RADIUS*cos(possible_angle_2)
            self.sParam.GoToPointP.y=GOAL_CENTER.y+ ARC_RADIUS*sin(possible_angle_2)
            self.sParam.finalslope=ballPos.angle(botPos)
            # print("me ",self.bot_id,"going to point", self.sParam.GoToPointP.x, self.sParam.GoToPointP.y)
            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return 