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
from copy import copy

ATTACKER_VISIBILITY_CONSTANT=5
SUPPORT_VISIBILITY_CONSTANT=0.25
SUPPORT_DISTANCE_CONSTANT=0.0001  
OPP_ANGLE_THRESH=15*pi/180
THRESH=15*pi/180                    # SUPPORT_DISTANCE_CONSTANT/fabs(dist-180)
BOT_DIAMETER=2*BOT_RADIUS
BOT_OPP_DIST_THRESH=350

class TMiddleDefender(Tactic):
    def __init__(self, bot_id, state, other_bot_id,param=None):
        self.attacker_id = bot_id
        self.other_bot_id = other_bot_id
        super(TMiddleDefender, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()


    # def bot_chooser(self,state):
    #     chosen_bots=[]
    #     my_botPos=Vector2D(int(state.homePos[self.attacker_id].x),int(state.homePos[self.attacker_id].y))
    #     for i in xrange(len(state.homePos)):
    #         if i==self.attacker_id: continue
    #         botPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
    #         if fabs(botPos.x) < HALF_FIELD_MAXX/3:
    #             chosen_bots.append([botPos,i])
    #     chosen_bots.sort(key= lambda item: fabs(item[0].dist(my_botPos)))
    #     chosen_bots=chosen_bots[:2]
    #     return chosen_bots



    def bisector_chooser(self, state, angle_to_shoot, bots):
        #assumuing bots[] will have the bot ids of just two elements.

        angle_to_shoot=copy(angle_to_shoot)
        my_bot_angles=[]
        botPos=Vector2D(int(state.homePos[self.attacker_id].x),int(state.homePos[self.attacker_id].y))
        for i in xrange(len(state.homePos)):
            if self.attacker_id==i or i not in bots: continue
            bot_coordinate=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            angle=Vector2D().normalizeAngle(bot_coordinate.angle(botPos)-state.homePos[self.bot_id].theta)
            my_bot_angles.append([angle,i])

        n=len(angle_to_shoot)
        n=n*(n-1)/2
        sum=inf
        now_bis=0
        corr_bis_and_bot=[]
        for _ in xrange(n):
            angle1=Vector2D().normalizeAngle(angle_to_shoot[now_bis]-my_bot_angles[0][0])
            for i in xrange(now_bis):
                angle2=Vector2D().normalizeAngle(angle_to_shoot[i]-my_bot_angles[1][0])
                sumhere=angle1+angle2
                if sumhere<sum:
                    sum=sumhere
                    corr_bis_and_bot=[[angle1,bots[0]],[angle2,bots[1]]]



            for i in xrange(now_bis+1,len(angle_to_shoot)):
                angle2=Vector2D().normalizeAngle(angle_to_shoot[i]-my_bot_angles[1][0])
                sumhere=angle1+angle2
                if sumhere<sum:
                    sum=sumhere
                    corr_bis_and_bot=[[angle1,bots[0]],[angle2,bots[1]]]

        return corr_bis_and_bot



    def execute(self, state, pub):
        supports=[]                                      #id, pos
        opponents=[]

        import sGoToPoint
        my_pos = Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        other_pos = Vector2D(int(state.homePos[self.other_bot_id].x), int(state.homePos[self.other_bot_id].y))

        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        ball_theta = atan((my_pos.y-ballPos.y)/ (my_pos.x  - ballPos.x))
        if(other_pos.dist(my_pos)<HALF_FIELD_MAXY):
            #print "*****************************MUST SEPARATE**********************"
            corner_up=Vector2D(int(0), int(HALF_FIELD_MAXY))
            if(corner_up.dist(my_pos)<corner_up.dist(other_pos)):
                    #print "bot no"+str(self.bot_id)+" going to corner_UP &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                    #self.sParam.GoToPointP.x=0
                    self.sParam.GoToPointP.y=HALF_FIELD_MAXY*3/5
                    self.sParam.GoToPointP.finalslope = ball_theta

                    sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            else:
                    #print "bot no "+str(self.bot_id)+"going to corner_DOWN ########################################"
                    #self.sParam.GoToPointP.x=0
                    self.sParam.GoToPointP.y=-HALF_FIELD_MAXY*3/5
                    self.sParam.GoToPointP.finalslope = ball_theta

                    sGoToPoint.execute(self.sParam, state, self.bot_id, pub)     
            return           



        def find_dark_region(att_pos, THRESH):
            opponents=[]
            for i in xrange(len(state.awayPos)):
                opp_pos=Vector2D(int(state.awayPos[i].x),int(state.awayPos[i].y))
                angle=opp_pos.normalizeAngle(opp_pos.angle(att_pos))
                opponents.append([i, opp_pos , angle, angle-THRESH, angle+THRESH])
            return opponents

        # supports=self.bot_chooser(state)

        sc1=sc2=sc3=0
        #self.attacker_id=0    
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y)) 
        my_id=self.bot_id
        DistancesFromBot = inf
        count=0
        for i in state.homePos:
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(ballPos)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                self.attacker_id=count
            count+=1
        #print "attacker is "+str(self.attacker_id)
             #attacker id
                                   #attacker bot_id
        
        botPos=Vector2D(int(state.homePos[self.attacker_id].x),int(state.homePos[self.attacker_id].y))
        opponents=[]                            #appending opponent posn, id, opp_angle wrt att
        supports=[]                             #appending support posn, id
        angles=[]                               #[0]->angle-th, [1]->angle+th, [2]->opp_id
        bis_angles=[]                           #[0]-> angle of bisectors wrt x-axis
        my_pos=Vector2D(int(state.homePos[my_id].x), int(state.homePos[my_id].y))
        my_angle=my_pos.normalizeAngle(my_pos.angle(botPos)-(state.homePos[self.attacker_id].theta))
        for i in xrange(len(state.awayPos)):
            opp_pos=Vector2D(int(state.awayPos[i].x),int(state.awayPos[i].y))
            angle=opp_pos.normalizeAngle(opp_pos.angle(botPos)-(state.homePos[self.attacker_id].theta))
            if i==self.attacker_id: continue
            opponents.append([opp_pos,i , angle])
        

           
        


        for i in xrange(len(state.homePos)):
            supports.append([Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)),i])  


        
        opponents.sort(key=lambda x:x[2])

        for opponent in opponents:
            angles.append([opponent[2],opponent[1]])
        Attack_BotPos = Vector2D(int(state.homePos[self.attacker_id].x), int(state.homePos[self.attacker_id].y))
        self.Opp_sorted_Angles=[]
        for angle in angles:
            ANGLE_THRESH = 2*acos((1.2*BOT_RADIUS)/ (Attack_BotPos.dist(Vector2D(int(state.homePos[angle[-1]].x), int(state.homePos[angle[-1]].y)))))
            self.Opp_sorted_Angles.append([angle[0]-ANGLE_THRESH,angle[0]+ANGLE_THRESH,angle[1]])
        
        lineangles=[]
        angle_gap=18.314*pi/180
        angle_to_shoot=[]
        i=0; j=1
        weights=[]
        ##print "Opp angles: "+str(self.Opp_sorted_Angles)
        for i in xrange(len(self.Opp_sorted_Angles)):
            origin_angle=self.Opp_sorted_Angles[i][1]
            j=i+1
            flag=0
            if j>=len(self.Opp_sorted_Angles): j=0; flag=1
            diff=self.Opp_sorted_Angles[j][0]-self.Opp_sorted_Angles[i][1]
            
            if flag:
                diff*=-1
                diff = 2*pi - diff


            if (diff>angle_gap) :
                angle_here=diff
                ##print "angle here inside: "+str(array(angle_here)*(180/pi))
                if angle_here>pi:
                    angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
                else:
                    angle_to_shoot.append(origin_angle+angle_here/2) 


               

        angle_to_shoot = map(botPos.normalizeAngle , angle_to_shoot)
        ##print("ANGLE TO SHOOT ..................."+str(array(angle_to_shoot)*(180/pi)))

        # bots=self.bot_chooser(state)
        bots = [self.bot_id, self.other_bot_id]
        best_bisectors_for_bots=self.bisector_chooser(state, angle_to_shoot, bots)  # each element contains [angle_of_bisector, attacker_id]
                                                                                          # where angle_of_bisector is relative



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
            if(fabs(y)>2000 or fabs(x)>1000):
               return 0
            return 1                         

        def assign_score(x, y):
            if(not valid_pos(x,y)):
                return -inf
            score=0
            # for support in supports:
            #     ##print str(support[1].x)+","+str(support[1].y)
            #     score+=SUPPORT_DISTANCE_CONSTANT/fabs(support[1].dist(Vector2D(int(x), int(y))))
            #     max_score=isvisible(Vector2D(int(x), int(y)), support[1], THRESH)
            #     row=[-1, 0, 1, 0]
            #     col=[0, 1, 0, -1]
            #     for k in range(4):
            #         ##print str(row[k])+" &&&&&&&&&&& "
            #         dest=Vector2D(int(support[1].x+ BOT_DIAMETER*row[k]), int(support[1].y+ BOT_DIAMETER*col[k]))
            #         curr_score=isvisible(Vector2D(int(x), int(y)), dest, THRESH)
            #         if(curr_score> max_score):
            #             max_score=curr_score
            #     score+=SUPPORT_VISIBILITY_CONSTANT*max_score
            score+=ATTACKER_VISIBILITY_CONSTANT*isvisible(att_pos, Vector2D(int(x), int(y)), THRESH)
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
            return best_pos         





        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y)) 
        att_id=0
        my_pos=Vector2D(int(state.homePos[self.bot_id].x), int(state.homePos[self.bot_id].y))
        if(not valid_pos(my_pos.x, my_pos.y)):
            self.sParam.GoToPointP.x=0
            self.sParam.GoToPointP.y=0
            self.sParam.GoToPointP.finalslope = ball_theta
            import sGoToPoint
            sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
            return 
        if(self.bot_id==supports[1][1]):
            other_bot_id=supports[0][1]
        else:
            other_bot_id=supports[1][1]
        other_pos=Vector2D(int(state.homePos[other_bot_id].x), int(state.homePos[other_bot_id].y))
               
        self.bots_in_black=[]
        self.bots_in_gray=[]
        botPos = Vector2D(int(state.homePos[self.attacker_id].x), int(state.homePos[self.attacker_id].y))
        for i in xrange(len(state.homePos)):
            if i==self.attacker_id: continue
            myBotPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            angle=myBotPos.angle(botPos)-state.homePos[self.attacker_id].theta
            ##print "angle for bot "+str(i)+" is "+str(angle*180/pi)
            for angles in self.Opp_sorted_Angles:
    
                if (angle>=angles[0] and angle<=angles[1])  :

                    dist_of_homeBot=myBotPos.dist(botPos)
                    dist_of_awayBot=Vector2D(int(state.awayPos[angles[2]].x),int(state.awayPos[angles[2]].y)).dist(botPos)
                   
                    if fabs(dist_of_awayBot) - fabs(dist_of_homeBot) > BOT_OPP_DIST_THRESH:
                        self.bots_in_black.append(i)

                    if fabs(dist_of_homeBot) - fabs(dist_of_awayBot) > BOT_OPP_DIST_THRESH:
                        self.bots_in_gray.append(i)

        all_bots_in_gray=True
        for i in supports:
            if i not in self.bots_in_gray:
                all_bots_in_gray=False
                break

        if all_bots_in_gray:
            #print "All bots in gray"
            for bot in best_bisectors_for_bots:
                if self.bot_id==bot[1] and self.bot_id not in self.bots_in_black:
                    x = botPos.x + OPTIMAL_DISTANCE * cos(bot[0])
                    y = botPos.y + OPTIMAL_DISTANCE * cos(bot[0])
                    best_pos, score = cover_the_region(x,y,5,-inf,bot[0])
                    import sGoToPoint
                    self.sParam.GoToPointP.x=best_pos.x
                    self.sParam.GoToPointP.y=best_pos.y
                    sGoToPoint.execute(self.sParam, state, self.bot_id, pub)
                    return

        #print "some bots in white!"
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
            if i is not self.bot_id:
                supports.append([i, Vector2D(int(state.homePos[i].x), int(state.homePos[i].y))])

        best_pos=cover_the_region(my_pos)

        self.sParam.GoToPointP.x=best_pos.x
        self.sParam.GoToPointP.y=best_pos.y
        self.sParam.GoToPointP.finalslope=ballPos.angle(best_pos)

        sGoToPoint.execute(self.sParam, state, self.bot_id, pub)

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

                    