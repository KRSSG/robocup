from tactic import Tactic
import time
import sys
from math import *
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')

from config import *
from numpy import array, inf

from geometry import * 
from math import *
import skills_union
import obstacle
import sGoToPoint
import sTurnToPoint

ATTACKER_VISIBILITY_CONSTANT=5
SUPPORT_VISIBILITY_CONSTANT=0.0
SUPPORT_DISTANCE_CONSTANT=0.0001  
GOAL_VISIBILITY_CONSTANT=25
OPP_ANGLE_THRESH=25*pi/180
THRESH=15*pi/180                    # SUPPORT_DISTANCE_CONSTANT/fabs(dist-180)
BOT_DIAMETER=2*BOT_RADIUS

class TForward(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TForward, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()

    def execute(self, state, pub):
        #ballPos=Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        supports=[]
        goal_center=Vector2D(int(HALF_FIELD_MAXX), int(0))
        def find_dark_region(att_pos, THRESH):
            opponents=[]
            for i in xrange(len(state.awayPos)):
                opp_pos=Vector2D(int(state.awayPos[i].x),int(state.awayPos[i].y))
                angle=opp_pos.normalizeAngle(opp_pos.angle(att_pos))
                opponents.append([i, opp_pos , angle, angle-THRESH, angle+THRESH])
            return opponents    

        def isvisible(source, destination, THRESH):
            visibility_angle=source.normalizeAngle(destination.angle(source))
            mina=source.normalizeAngle(visibility_angle-THRESH)
            maxa=source.normalizeAngle(visibility_angle+THRESH)
            opponents=find_dark_region(source, THRESH)
            if(maxa-mina<0):
                angle_gap1=angle_gap2=0
                for opponent in opponents:
                    if(opponent[2]>mina or opponent[2]<maxa):
                        if(opponent[2]>mina):
                            angle_gap1=opponent[2]-mina
                        else:
                            angle_gap2=maxa- opponent[2]
                        return (-min(angle_gap1, angle_gap2))        
                return 1
            else:
                for opponent in opponents:
                    if(mina<opponent[2]<maxa):
                        return (-(min(opponent[2]-mina, maxa- opponent[2])))
                return 1      

        def valid_pos(x, y):
            if(fabs(y)>2000 or (x)<1000 or fabs(x)>2900):
               return 0
            if(fabs(y)<750 and x>2450):
               return 0   
            return 1                         

        def assign_score(x, y):
            if(not valid_pos(x,y)):
                return -inf
            score=0
            for support in supports:
                #print str(support[1].x)+","+str(support[1].y)
                score+=SUPPORT_DISTANCE_CONSTANT/fabs(support[1].dist(Vector2D(int(x), int(y))))
                max_score=isvisible(Vector2D(int(x), int(y)), support[1], THRESH)
                row=[-1, 0, 1, 0]
                col=[0, 1, 0, -1]
                for k in range(4):
                    #print str(row[k])+" &&&&&&&&&&& "
                    dest=Vector2D(int(support[1].x+ BOT_DIAMETER*row[k]), int(support[1].y+ BOT_DIAMETER*col[k]))
                    curr_score=isvisible(Vector2D(int(x), int(y)), dest, THRESH)
                    if(curr_score> max_score):
                        max_score=curr_score
                score+=SUPPORT_VISIBILITY_CONSTANT*max_score
            score+=ATTACKER_VISIBILITY_CONSTANT*isvisible(att_pos, Vector2D(int(x), int(y)), THRESH)
            score+=GOAL_VISIBILITY_CONSTANT*isvisible(Vector2D(int(x), int(y)), goal_center, THRESH)
            return score

        def cover_the_region(my_pos):
            row=[-1, 0, 1,  0]
            col=[ 0, 1, 0, -1]
            max_score=assign_score(my_pos.x, my_pos.y)
            best_pos=my_pos
            for k in range(4):
                curr_score=assign_score(my_pos.x+BOT_DIAMETER*row[k], my_pos.y+BOT_DIAMETER*col[k])
                if(curr_score>max_score):
                    max_score=curr_score
                    best_pos=Vector2D(int(my_pos.x+BOT_DIAMETER*row[k]), int(my_pos.y+BOT_DIAMETER*col[k]))
            #print "MAX_SCORE= "+str(max_score)        
            return best_pos         


        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y)) 
        att_id=0
        my_pos=Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        if(not valid_pos(my_pos.x, my_pos.y)):
            self.sParam.GoToPointP.x=2000
            self.sParam.GoToPointP.y=500
            self.sParam.GoToPointP.finalslope=0
            #print "BEFORE 108 bot_id="+str(self.bot_id)
            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return 

        DistancesFromBot = inf
        count=0
        for i in state.homePos:
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(ballPos)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                att_id=count
            count+=1
        att_pos=Vector2D(int(state.homePos[att_id].x), int(state.homePos[att_id].y))
        opponents=find_dark_region(att_pos, OPP_ANGLE_THRESH)                               #id, pos, angle, angle-thresh, angle+thresh
        for i in xrange(len(state.homePos)):
            if(not (i==self.bot_id or i==4)):
                supports.append([i, Vector2D(int(state.homePos[i].x), int(state.homePos[i].y))])


              
        best_pos=cover_the_region(my_pos)
        #print " CURR_POS="+str(my_pos.x)+","+str(my_pos.y)
        #print " BEST_POS="+str(best_pos.x)+","+str(best_pos.y)

        self.sParam.GoToPointP.x=best_pos.x
        self.sParam.GoToPointP.y=best_pos.y
        self.sParam.GoToPointP.finalslope=ballPos.angle(best_pos)

        sGoToPoint.execute(self.sParam, state, self.bot_id, pub)


        # self.sParam.TurnToPointP.x=state.ballPos.x
        # self.sParam.TurnToPointP.y=state.ballPos.y
        # self.sParam.TurnToPointP.max_omega=MAX_BOT_OMEGA;

        # sTurnToPoint.execute(self.sParam, state, self.bot_id, pub)    
    def isComplete(self, state):
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        # No parameter to update here
        pass    