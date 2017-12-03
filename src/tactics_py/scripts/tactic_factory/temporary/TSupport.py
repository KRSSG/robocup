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


K_OP_DIST=500000/HALF_FIELD_MAXX
K_SUPP_DIST=1.0/HALF_FIELD_MAXX
SUPPORT_WEIGHT =  .1 
OPPONENT_WEIGHT =  3
K_ANGLE_DIFF=pi/16
BIS_ANGLE_WEIGHT=.3
#ANGLE_THRESH=10*pi/180
OPTIMAL_DISTANCE_WEIGHT=.3
MIN_HALF_BIS_ANGLE=8*pi/180
N_ITER=1
BOT_DIAMETER=2*BOT_RADIUS
SUPP_OPP_DIST_THRESH=BOT_RADIUS
OPTIMAL_DISTANCE = 3.0*HALF_FIELD_MAXX/5
OPTIMAL_DISTANCE_CONST = OPTIMAL_DISTANCE/45




from geometry import * 
from math import *
import skills_union
import obstacle
import sGoToPoint

class TSupport(Tactic):
    def __init__(self, bot_id, state, param=None):
        super(TPosition, self).__init__( bot_id, state, param)
        self.sParam = skills_union.SParam()

    def angle_1_wrt_2(self, p1, p2):                  #p1 & p2 ->Vector2D
        angle_calc=0
        angle_calc=atan2((p1.y-p2.y),(p1.x-p2.x))
        #print "angle of "+str(p1.x)+","+str(p1.y)+" wrt "+str(p2.x)+","+str(p2.y)+" is "+str(angle_calc*180/pi)

        return angle_calc

    def bot_chooser(self,state):
        chosen_bots=[]
        my_botPos=Vector2D(int(state.homePos[self.attacker_id].x),int(state.homePos[self.attacker_id].y))
        for i in xrange(len(state.homePos)):
            if i==self.attacker_id: continue
            botPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            if fabs(botPos.x) < HALF_FIELD_MAXX/3:
                chosen_bots.append([botPos,i])
        chosen_bots.sort(key= lambda item: fabs(item[0].dist(my_botPos)))
        chosen_bots=chosen_bots[:2]
        return chosen_bots



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

            # last_angle=angle_to_shoot[-1]
            # angle_to_shoot=angle_to_shoot[:-1]
            # angle_to_shoot=[last_angle]+angle_to_shoot
            # sumhere=0
            # for j in xrange(len(my_bot_angles)):
            #     sumhere+=fabs(my_bot_angles[j][0] - angle_to_shoot[j])
            # if sumhere<sum:
            #     sum=sumhere
            #     corr_bis_and_bot=[]
            #     for j in xrange(len(my_bot_angles)):
            #         corr_bis_and_bot.append([angle_to_shoot[j],my_bot_angles[j][1]])

        return corr_bis_and_bot

    def isForward(self, state, bots):
        the_bot_x=-inf
        the_bot=-1
        for i in xrange(len(state.homePos)):
            if i==self.attacker_id: continue
            flag=False
            for j in bots:
                if i==j[0]: flag=True; break
            if flag: continue
            if state.homePos.x>the_bot_x:
                the_bot=i

        bots.append([Vector2D(int(state.homePos[the_bot].x),int(state.homePos[the_bot].y)),the_bot])
        return the_bot



    def execute(self, state, pub):
        #print "HELLO**********************************************************************************************************************"
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
        print "attacker is "+str(self.attacker_id)
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
            opponents.append([opp_pos,i , angle])
        

           
        


        for i in xrange(len(state.homePos)):
            supports.append([Vector2D(int(state.homePos[i].x),int(state.homePos[i].y)),i])  


        
        opponents.sort(key=lambda x:x[2])

        for opponent in opponents:
            angles.append([opponent[2],opponent[1]])
        Attack_BotPos = Vector2D(int(state.homePos[self.attacker_id].x), int(state.homePos[self.attacker_id].y))
        self.Opp_sorted_Angles=[]
        for angle in angles:
            ANGLE_THRESH = 2*acos((1.2*BOT_RADIUS)/ (Attack_BotPos.dist(Vector2D(int(state.homePos[angle[-1]]), int(state.homePos[angle[-1]])))))
            self.Opp_sorted_Angles.append([angle[0]-ANGLE_THRESH,angle[0]+ANGLE_THRESH,angle[1]])
        
        lineangles=[]
        angle_gap=18.314*pi/180
        angle_to_shoot=[]
        i=0; j=1
        weights=[]
        #print "Opp angles: "+str(self.Opp_sorted_Angles)
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
                #print "angle here inside: "+str(array(angle_here)*(180/pi))
                if angle_here>pi:
                    angle_to_shoot.extend([origin_angle+(angle_here/3),origin_angle+(angle_here*2/3)])
                else:
                    angle_to_shoot.append(origin_angle+angle_here/2) 


               

        angle_to_shoot = map(botPos.normalizeAngle , angle_to_shoot)
        #print("ANGLE TO SHOOT ..................."+str(array(angle_to_shoot)*(180/pi)))

        bots=self.botChooser(self,state)
        
        best_bisectors_for_bots=self.bisector_choooser(self,state, angle_to_shoot, bots)  # each element contains [angle_of_bisector, attacker_id]
                                                                                          # where angle_of_bisector is relative

        # closest_bis_angle=angle_to_shoot[0]
        # for angle in angle_to_shoot:
        #     if(fabs(angle-my_angle)<(closest_bis_angle- my_angle)):
        #         closest_bis_angle=angle


        self.bots_in_black=[]
        self.bots_in_gray=[]
        for i in xrange(len(state.homePos)):
            if i==self.attacker_id: continue
            myBotPos=Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
            angle=myBotPos.angle(botPos)-state.homePos[self.attacker_id].theta
            #print "angle for bot "+str(i)+" is "+str(angle*180/pi)
            for angles in self.Opp_sorted_Angles:
    
                if (angle>=angles[0] and angle<=angles[1])  :

                    dist_of_homeBot=myBotPos.dist(botPos)
                    dist_of_awayBot=Vector2D(int(state.awayPos[angles[2]].x),int(state.awayPos[angles[2]].y)).dist(botPos)
                   
                    if fabs(dist_of_awayBot) - fabs(dist_of_homeBot) > BOT_OPP_DIST_THRESH:
                        self.bots_in_black.append(i)

                    if fabs(dist_of_homeBot) - fabs(dist_of_awayBot) > BOT_OPP_DIST_THRESH:
                        self.bots_in_gray.append(i)
        
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

        forward_bot = self.isForward(state, bots)
        defence_bot = self.isDefence(state,bots)
                


        self.free_angles = []
        

        sx=my_pos.x
        sy=my_pos.y

        def cover_the_region(cx, cy, iter, mscore, closest_bis_angle):
            #print "iter= "+str(iter)+">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
            row=[-1, 0,1,0,-1,1,-1,1]
            col=[ 0,-1,0,1,-1,1,1,-1]
            index=-1
            cscore=0
            for k in range(8):
                sc=assign_score(cx+row[k]*BOT_DIAMETER,cy+col[k]*BOT_DIAMETER, closest_bis_angle)
                cscore=(sc[0]+sc[1]+sc[2]+sc[3])
                # print "In co-ordinate = "+str(cx+BOT_DIAMETER*row[k])+str(cy+BOT_DIAMETER*col[k])
                # print "sc1 (opp_dist)="+str(sc[0])
                # print "sc2 (supp_dist)="+str(sc[1])
                # print "sc3 (angle_bis)="+str(sc[2])
                # print "sc4 (optimal_dist)="+str(sc[3])
                if(cscore>mscore):
                    mscore=cscore
                    index=k
            if(index==-1): 
                #print "...................******************************************************.mscore="+str(mscore)
                return [Vector2D(int(cx), int(cy)),mscore]
            else:
                return cover_the_region(cx+BOT_DIAMETER*row[index], cy+BOT_DIAMETER*col[index], iter+1, mscore, closest_bis_angle)            

        def out_of_field(x,y):
            if((x>HALF_FIELD_MAXX or x<-HALF_FIELD_MAXX) or (y>HALF_FIELD_MAXY or y<-HALF_FIELD_MAXY)):
                return 1
            return 0    
        def score_dist_opp(dista):
            #print "for opponent part dist="+str(dista)+" score="+str(AWAY_BOT_APP_GWT*exp(+K_OP_DIST*dista))+"  "+str(dista)
            #return 0
            return -1*(K_OP_DIST/dista)

        def score_dist_supp(dist): #*********yet to add for support*************
            #print "for support part dist="+str(dist)+" score= "+str(OUR_BOT_APPROACHABILITY_CONST*exp(-K_SUPP_DIST*dist))
            #print "exponent part = "+str(exp(-K_SUPP_DIST*dist))+ "where dist*K ="+str(-K_SUPP_DIST*dist)
            return 0
            return SUPPORT_WEIGHT*exp(-K_SUPP_DIST*dist)

        def bis_angle_diff(ang_diff):
            return 0
            return K_ANGLE_DIFF/ang_diff   

        def dist_from_attacker(distance):
            #return 0
            diff=fabs(OPTIMAL_DISTANCE - distance)
            #print "att_pos="+str(botPos.x)+","+str(botPos.y)
            #print "********************************************current dist="+str(distance)
            #print "OPTIMAL_DIST="+str(OPTIMAL_DISTANCE)
            # normalizeDistance = OPTIMAL_DISTANCE_WEIGHT * exp
            #print "distance from OPTIMAL_DIST"+str(diff)
            return OPTIMAL_DISTANCE_CONST/diff

        def assign_score(x , y, closest_bis_angle):         #p1: opponent dist, p2: support dist, p3: safe region & angle bis
            flag=1; 
            sc1=sc2=sc3=sc4=0
            #print "############################checking x="+str(x)+" HALF_FIELD_MAXX="+str(HALF_FIELD_MAXX)
            #print "############################checking y="+str(y)+" HALF_FIELD_MAXY="+str(HALF_FIELD_MAXY)
            if(out_of_field(x,y)):
                #print "&&&&&&&&&&&&&&&&&&&&&&&&&& OUT OF FIELD"
                return [-fabs(fabs(x)- HALF_FIELD_MAXX),-fabs(fabs(y)- HALF_FIELD_MAXY),-1,-1]
            p=[]
            cell_pos=Vector2D(int(x),int(y))
            this_angle=my_pos.normalizeAngle(cell_pos.angle(botPos)-(state.homePos[self.attacker_id].theta))
            this_pos=Vector2D(int(x),int(y))
            for angle in self.Opp_sorted_Angles:
                #print "checking "+str(x)+","+str(y)+" between "+str(angle[0]*180/pi)+","+str(angle[1]*180/pi)+" my_angle="+str(this_angle*180/pi)
                if(angle[0]<this_angle<angle[1]):
                   # for opponent in opponents:
                    opp_dist=Vector2D(int(state.awayPos[angle[2]].x), int(state.awayPos[angle[2]].y)).dist(botPos)
                    if(opp_dist<this_pos.dist(supports[self.attacker_id][0])+ SUPP_OPP_DIST_THRESH):
                            #print "caught in black region ,opponent no"+str(angle[2])+" close to attacker between angle="+str(angle[0]*180/pi)+","+str(angle[1]*180/pi)+" my_angle="+str(this_angle*180/pi)  
                            #print "opponent"+str(state.awayPos[angle[2]].x)+","+str(state.awayPos[angle[2]].y)+" is at dist="+str(opp_dist)
                            #print "cell_pos "+str(x)+","+str(y)+" is at dist="+str(this_pos.dist(supports[self.attacker_id][0]))
                            flag=0
                            break
            if(flag==0):                        #if in black region & behind opp, score=0
                #print "caught in black region *******************************************************************************"
                return [0,0,0,0]

            for opponent in opponents:
                sc1+=score_dist_opp(opponent[0].dist(this_pos))
            #print "for all opp sc1="+str(sc1)
            for support in supports:
                sc2+=score_dist_supp(support[0].dist(this_pos))

            #print "for all support sc2="+str(sc2)    
            
            sc3=bis_angle_diff(fabs(this_angle-closest_bis_angle))
            #print "cell_pos="+str(cell_pos.x)+","+str(cell_pos.y)
            #print "bot_pos="+str(botPos.x)+","+str(botPos.y)
            #print "dist="+str(cell_pos.dist(botPos))
            sc4=dist_from_attacker(cell_pos.dist(botPos))
            p.append(sc1)
            p.append(sc2)
            p.append(sc3)
            p.append(sc4)
            #print " p[0]= "+str(p[0])
            #print p
            return p
        ######################
       
        # this_bot_angle=self.angle_1_wrt_2(supports[self.attacker_id][0],supports[self.attacker_id][0])
        # closest_bis_angle=bis_angles[0];
        # for bise in bis_angles:
        #     if(fabs(bise- this_bot_angle)<fabs(closest_bis_angle- this_bot_angle)):
        #         closest_bis_angle=bise;
        
        # distance= supports[self.attacker_id][0].dist(supports[self.attacker_id][0])
        # sx=distance*cos(closest_bis_angle)
        # sy=distance*sin(closest_bis_angle)      
        sc=assign_score(sx,sy, closest_bis_angle)

        # #print "sc[0]="+str(sc[0])
        # #print "wehfeicfheicfjerkfjfje: "+str(sc)
        cscore=(sc[0])+(sc[1])+(sc[2]+sc[3])
        # #print "cscore= "+str(cscore)
        best_pos=cover_the_region(sx, sy, 0, cscore, closest_bis_angle)
        print "closest_angle_bis="+str(closest_bis_angle*180/pi)
        print "best_pos= "+str(best_pos[0].x)+","+str(best_pos[0].y)
        print "max_score="+str(best_pos[1])
        if(best_pos[1]==0):
            self.sParam.GoToPointP.x=botPos.x+OPTIMAL_DISTANCE*cos(closest_bis_angle+state.homePos[self.attacker_id].theta)
            self.sParam.GoToPointP.y=botPos.y+OPTIMAL_DISTANCE*cos(closest_bis_angle+state.homePos[self.attacker_id].theta)
        else:
            self.sParam.GoToPointP.x=best_pos[0].x
            self.sParam.GoToPointP.y=best_pos[0].y        
        #print "****************************************************************************************************"

        
        self.sParam.GoToPointP.finalslope=0#self.angle_1_wrt_2(supports[self.attacker_id][0], best_pos)
        self.sParam.GoToPointP.align=1
        self.sParam.GoToPointP.finalVelocity=0
        #print "going to call GoToPointP skill. self.attacker_id="+str(self.attacker_id)
        #
        # print "closest_bis_angle="+str(closest_bis_angle*180/pi)
        # print "sc1="+str(sc[0])
        # print "sc2="+str(sc[1])
        # print "sc3="+str(sc[2])
       
        sGoToPoint.execute(self.sParam, state, my_id, pub)
        


    def isComplete(self, state):
        # TO DO use threshold distance instead of actual co ordinates
        if self.destination.dist(state.homePos[self.attacker_id]) < self.threshold:
            return True
        elif time.time()-self.begin_time > self.time_out:
            return True
        else:
            return False

    def updateParams(self, state):
        # No parameter to update here
        pass

        
         




