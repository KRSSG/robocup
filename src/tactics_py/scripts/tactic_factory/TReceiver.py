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
import sTurnToAngle
import sDribble
import sVelocity
import sMoveWithDribbler

class TReceiver(Tactic):
    def __init__(self,bot_id,state,params=None):
        super(TReceiver,self).__init__(bot_id,state,params)
        self.botid=int(bot_id)
        self.sParams=skills_union.SParam()

    def execute(self,state,pub,attacker_id):
        if self.botid <0:
            return
        # print "hasdfsadfasdfas"
        # file  = open("debugging.txt","a")
        # print("#"*10+"\n")
        Point=Vector2D()
        ballPos=Vector2D(int(state.ballPos.x),int(state.ballPos.y))
        self.botPos = Vector2D(int(state.homePos[self.botid].x) , int(state.homePos[self.botid].y) )
      

        bot_closest_to_ball = attacker_id
        BotClosestPos=Vector2D(int(state.homePos[bot_closest_to_ball].x),int(state.homePos[bot_closest_to_ball].y))
        # angle1 =  self.botPos.angle(BotClosestPos)

        # def ball_angle(self):
        #     ballVel=Vector2D(int(state.ballVel.x),int(state.ballVel.y))
        #     ballangle = atan2(ballVel.y,ballVel.y)
        #     return ballangle
        # if state.ballVel.x ==0 and state.ballVel.y ==0:
        #     print("New idea\n")
        #     print "New idea"
        #     delta_y = state.ballPos.y - BotClosestPos.y
        #     delta_x = state.ballPos.x - BotClosestPos.x
        #     angle2 = atan2(delta_y,delta_x)
        # else:
        #     print("Old idea\n")
        #     print "oold idea"
        #     angle2 = ball_angle(self)

        # print("New idea\n")
        # print "New idea"
        delta_y = state.ballPos.y - BotClosestPos.y
        delta_x = state.ballPos.x - BotClosestPos.x
        if ballPos.dist(BotClosestPos) < DRIBBLER_BALL_THRESH:
            angle2 = atan2(delta_y,delta_x)
        else:
            angle2 = atan2(state.ballVel.y,state.ballVel.x + 0.0001)
        # anglediff = Point.normalizeAngle(angle1 - angle2)

        # r = self.botPos.dist(BotClosestPos)

        # destination = Vector2D()
        # destination.x = int(BotClosestPos.x + r*cos(angle2))
        # destination.y = int(BotClosestPos.y + r*sin(angle2))
        # self.sParams.GoToPointP.x = destination.x
        # self.sParams.GoToPointP.y = destination.y
        # self.sParams.GoToPointP.finalVelocity = 0
        # self.sParams.GoToPointP.finalslope = pi - angle2
        botballdist = self.botPos.dist(ballPos)
        angle1 = self.botPos.angle(ballPos)
        # anglediff = Point.normalizeAngle(angle1 - angle2)
        anglediff = angle1 - angle2
        r = self.botPos.dist(ballPos) * cos(anglediff)
        # prep = self.botPos.dist(ballPos) * sin(anglediff)
        # print ("attacker by my tactic == " + str(bot_closest_to_ball) + "\n") 
        # print ("Receiver by my tactic == " + str(self.botid)+"\n")

        finalslope = pi + ballPos.angle(Vector2D(int(state.homePos[self.botid].x),int(state.homePos[self.botid].y)))
        # if angle2>=0:
        #     finalslope = -1*fabs(finalslope)
        ob =Vector2D()
        angletoturn = ob.normalizeAngle(finalslope - state.homePos[self.botid].theta)
        # angletoturn = BotClosestPos.normalizeAngle(angle2 + pi - state.homePos[self.botid].theta)
        # finalslope = pi - angle2

        #stackoverflow soltion

        k = (state.ballPos.y - BotClosestPos.y)*(state.homePos[self.botid].x - BotClosestPos.x)
        k -= (state.ballPos.x - BotClosestPos.x)*(state.homePos[self.botid].y - BotClosestPos.y)

        k /= (pow(state.ballPos.y  - BotClosestPos.y ,2) + pow(state.ballPos.x - BotClosestPos.x ,2))

        x = state.homePos[self.botid].x - k*(state.ballPos.y - BotClosestPos.y)
        y = state.homePos[self.botid].y + k*(state.ballPos.x - BotClosestPos.x)

        prep = self.botPos.dist(Vector2D(int(x),int(y)))
        r = ballPos.dist(Vector2D(int(x),int(y)))
        if prep > 0.1*BOT_BALL_THRESH and prep < 4*BOT_RADIUS :
            ballVel =Vector2D(int(state.ballVel.x),int(state.ballVel.y))
            destination = Vector2D()
            # destination.x = int(ballPos.x + r*cos(angle2))
            # destination.y = int(BotClosestPos.y + r*sin(angle2))
            destination.x = int(x)
            destination.y = int(y)
            time_ball = r / (destination.abs(ballVel)+0.0001)
            time_bot = prep / MAX_BOT_SPEED

            # print("time->ball,bot,distance",time_ball,time_bot,prep,r)
            # if time_ball < 1.3 * time_bot:
            #     # print "time manage"
            #     r = r*1.4
            #     destination.x = int(ballPos.x + r*cos(angle2))
            #     destination.y = int(BotClosestPos.y + r*sin(angle2))


            # print ("angle2 == "+ str(angle2))


            
            # print ("destination.x " + str(destination.x))
            # print ("destination.y " + str(destination.y))
            print("going normally")
            self.sParams.GoToPointP.x = destination.x
            self.sParams.GoToPointP.y = destination.y
            self.sParams.GoToPointP.finalVelocity = 0
            self.sParams.GoToPointP.finalslope =  finalslope
            self.sParams.GoToPointP.align = True
            # self.sParams.GoToPointP.align =False
            # print ("gotopoint ",destination.x,destination.y)
            sGoToPoint.execute(self.sParams,state,self.botid,pub,True)
            return

        # elif prep>4*BOT_RADIUS:
        #     threshold = 0.5
        #     print("GOalie")
        #     destination=Vector2D()
        #     r = ballPos.dist(Vector2D(int(state.homePos[self.botid].x),int(state.homePos[self.botid].y)))
        #     destination.x = int(ballPos.x + r*cos(threshold))
        #     destination.y = int(ballPos.y + r*sin(threshold))
        #     self.sParams.GoToPointP.x = destination.x
        #     self.sParams.GoToPointP.y = destination.y
        #     self.sParams.GoToPointP.finalVelocity = 0
        #     self.sParams.GoToPointP.finalslope =  finalslope
        #     self.sParams.GoToPointP.align = True
        #     # self.sParams.GoToPointP.align =False
        #     # print ("gotopoint ",destination.x,destination.y)
        #     sGoToPoint.execute(self.sParams,state,self.botid,pub)
        #     return
    
        if angletoturn > SATISFIABLE_THETA : 
            print (angletoturn,SATISFIABLE_THETA/2)
            print ("aligning\n")
            print "align"
            # import sTurnToPoint
            # self.sParams.TurnToPointP.x = state.homePos[attacker_id].x
            # self.sParams.TurnToPointP.y = state.homePos[attacker_id].y
            # self.sParams.TurnToPointP.max_omega = angletoturn/(pi)
            # sTurnToPoint.execute(self.sParams,state,self.botid,pub)
            # return
            self.sParams.TurnToAngleP.finalslope = finalslope
            sTurnToAngle.execute(self.sParams,state,self.botid,pub,True)
            return
 
        # if botballdist < 6*DRIBBLER_BALL_THRESH:
        #     import sDribble
        #     print("dribble")
        #     sDribble.execute(self.sParams,state,self.botid,pub)
        elif prep < BOT_RADIUS:
            # if botballdist < 2 * DRIBBLER_BALL_THRESH:
            #     print ("Dribbling\n")
            #     sDribble.execute(self.sParams,state,self.botid,pub)
            # print "botballdist = " + str(botballdist)
            # print("botballdist = " + str(botballdist)+"\n")
            # print ("in action\n\n")
            print ("moving in direction of ball\n")
            speed = ((state.ballVel.x**2 + state.ballVel.y**2)**0.5)/3
            speed=800
            print("speed",speed)
            motionangle = ballPos.angle(Vector2D(int(state.homePos[self.botid].x),int(state.homePos[self.botid].y)))
            print(angle2,prep)
            self.sParams.MoveWithDribblerP.v = speed
            self.sParams.MoveWithDribblerP.theta = motionangle
            # self.sParams.MoveWithDribblerP.v_t = 0
            sMoveWithDribbler.execute(self.sParams,state,self.botid,pub)
            return
        else:
            # import sDribble
            # sDribble.execute(self.sParams,state,self.botid,pub)
             self.sParams.GoToPointP.x = x
             self.sParams.GoToPointP.y = y
             self.sParams.GoToPointP.finalVelocity = 0
             self.sParams.GoToPointP.finalslope =  finalslope
             self.sParams.GoToPointP.align = True
             # self.sParams.GoToPointP.align =False
             # print ("gotopoint ",destination.x,destination.y)
             sGoToPoint.execute(self.sParams,state,self.botid,pub)
             return
            # if botballdist < 1600:
            #     print ("in action\n\n")
            #     print ("moving in direction of ball\n")
            #     self.sParams.MoveWithDribblerP.v_x = state.ballVel.x /3
            #     self.sParams.MoveWithDribblerP.v_y = state.ballVel.y/3
            #     self.sParams.MoveWithDribblerP.v_t = 0
            #     sMoveWithDribbler.execute(self.sParams,state,self.botid,pub)
            # else:
            #     print "ball dist inf"
            #     print ("ball dist inf\n")

        # file.close()
        # print "skill test"
        # self.sParams.MoveWithDribblerP.v_x = state.ballVel.x /3
        # self.sParams.MoveWithDribblerP.v_y = state.ballVel.y/3
        # self.sParams.MoveWithDribblerP.v_t = 0
        # sMoveWithDribbler.execute(self.sParams,state,self.botid,pub)

    def isComplete(self, state):
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.bot_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        pass