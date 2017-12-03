from tactic import Tactic
import time
import sys

sys.path.append('/home/mehul/Documents/KRSSG/src/skills_py/scripts/skills')
sys.path.append('/home/mehul/Documents/KRSSG/src/plays_py/scripts/utils/')
sys.path.insert(0, '/home/mehul/Documents/KRSSG/src/navigation_py/scripts/navigation')

from geometry import Vector2D 
import skills_union
'''
import sGoToPoint
import sGoToBall
import sDribbleTurn
'''
import obstacle
import sDribbleTurn
import sDefendPoint
import sGoToPoint
#import sTurnToPoint
import sKickToPoint

class TPosition(Tactic):

    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()

        #self.destination = Vector2D(int(self.param.PositionP.x), int(self.param.PositionP.y))
        self.threshold = 20.0

    def execute(self, state, pub):
        '''
        for sDribbleTurn--------------------------------------------------
        self.sParam.DribbleTurnP.x             = 150 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 150#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 15  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 25 #self.param.PositionP.finalSlope = 0
        '''

        #for sGoToBall----------------------------------------------------         
        #self.sParam.GoToBallP.intercept      = False #true or false ask?

        
        '''
        #for sTurnToPoint------------------------------------------------
        self.sParam.TurnToPointP.x             = 200 #self.param.PositionP.x 
        self.sParam.TurnToPointP.y             = 800 #self.param.PositionP.y 
        self.sParam.TurnToPointP.max_omega     = 10  #self.param.PositionP.align = False

        

        #for sDefendPoint-------------------------------------------------
        self.sParam.DefendPointP.x             = 150 #self.param.PositionP.x 
        self.sParam.DefendPointP.y             = 150#self.param.PositionP.y 
        self.sParam.DefendPointP.finalslope    = 15  #self.param.PositionP.align = False
        self.sParam.DefendPointP.radius        = 1.2 #self.param.PositionP.finalSlope = 0

        

        #for sKickToPoint---------------------------------------------------
        self.sParam.KickToPointP.x             = 0
        self.sParam.KickToPointP.y             = 0
        self.sParam.KickToPointP.power         = 50
        '''
        '''
        self.sParam.DribbleTurnP.x             = -1200 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 2000#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 3  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 50 #self.param.PositionP.finalSlope = 0
        
        self.sParam.KickToPointP.x             = 1200
        self.sParam.KickToPointP.y             = -700
        self.sParam.KickToPointP.power         = 50
        '''

        self.sParam.DribbleTurnP.x             = 150 #self.param.PositionP.x 
        self.sParam.DribbleTurnP.y             = 150#self.param.PositionP.y 
        self.sParam.DribbleTurnP.max_omega     = 15  #self.param.PositionP.align = False
        self.sParam.DribbleTurnP.turn_radius   = 5 #self.param.PositionP.finalSlope = 0
       

        sDribbleTurn.execute(self.sParam, state, self.bot_id, pub)

    def isComplete(self, state):
        botpos = Vector2D(int(state.homePos[bot_id].x),int(state.homePos[bot_id].y))
        ballpos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        dis_from_point = math.sqrt(math.pow(ballpos.x-botpos.x,2)+math.pow(ballpos.y-botpos.y,2))
        angle = ballpos.normalizeAngle(ballpos.angle(botpos) - (state.homePos[botID].theta))

        if (dis_from_point < 3*BOT_POINT_THRESH) and (math.fabs(angle)<0.2):
            return True
        else:
            return False

        '''
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False
        '''

    def updateParams(self, state):
        # No parameter to update here
        pass
