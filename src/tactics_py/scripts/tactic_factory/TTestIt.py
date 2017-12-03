from tactic import Tactic
import time
import sys

sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
import sStop
import sKickToPoint
import sGoToPoint
import skills_union

class TTestIt(Tactic):

    def __init__(self, bot_id, state, param=None):
        super(TTestIt, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()
        self.flag=1
        # TODO: Need to set these threshold velocity values
        self.vel_x_threshold = 0.0
        self.vel_y_threshold = 0.0

    def execute(self, state, pub, flag):
        self.sParam.GoToPointP.y=0
        
        if(state.homePos[1].x>2800):
            flag[0]=1
        elif (state.homePos[1].x<-2800):
            flag[0]=0    
        if(flag[0]==1):
            print("going to -3000")
            self.sParam.GoToPointP.x=-3000
            sGoToPoint.execute(self.sParam, state, 1, pub)
            return
        if(flag[0]==0):
            print("going to +3000")
            self.sParam.GoToPointP.x=3000
            sGoToPoint.execute(self.sParam, state, 1, pub)

    def isComplete(self, state):
        if state.homeVel[self.bot_id].x <= self.vel_x_threshold and state.homeVel[self.bot_id].y <= self.vel_y_threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        # No parameter to update here
        pass
