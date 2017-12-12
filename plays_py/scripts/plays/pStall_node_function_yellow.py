import sys
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
from config import *
from geometry import *
from numpy import inf
from math import fabs


def assign_roles_stall(state):

    # if fabs(state.ballPos.x) > HALF_FIELD_MAXX or fabs(state.ballPos.y) > HALF_FIELD_MAXY:
    #     return 0

    goal_center=Vector2D(int(-HALF_FIELD_MAXX), int(0))
    field_center=Vector2D(int(0), int(0))
    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y)) 
    DistancesFromBot = inf
    assigned_roles=[]
    att_id=0
    goalie_id=4
    count=0
    for i in state.homePos:
      if(count==goalie_id):
        count+=1
        continue
      else:     
        botIDs=Vector2D(int(i.x),int(i.y))
        dist=botIDs.dist(ballPos)
        if dist<DistancesFromBot:
            DistancesFromBot=dist
            att_id=count
        count+=1

    assigned_roles.append(att_id)
    DistancesFromBot = inf
    count=0
    primary_defender_id=-1
    for i in state.homePos:
        if(count==att_id or count==goalie_id):
            count+=1
            continue
        else:
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(goal_center)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                primary_defender_id=count
        count+=1    
    assigned_roles.append(primary_defender_id)

    middle_defender1_id=-1
    middle_defender2_id=-1
    forward_id=-1
    DistancesFromBot = inf
    DistancesFromBot1=inf
    count=0
    for i in state.homePos:
        #print "count="+str(count)
        if(count==att_id or count==goalie_id or count==primary_defender_id):
            count+=1
            continue
        else:
            #print "in else checking for "+str(count)
            botIDs=Vector2D(int(i.x),int(i.y))
            dist=botIDs.dist(field_center)
            if dist<DistancesFromBot:
                DistancesFromBot=dist
                middle_defender2_id=middle_defender1_id
                middle_defender1_id=count
            else:
                if(dist<DistancesFromBot1):
                    DistancesFromBot1=dist
                    middle_defender2_id=count
                  
        count+=1    
    assigned_roles.append(middle_defender1_id)
    assigned_roles.append(middle_defender2_id)

    for i in range(6):
        if( i not in [goalie_id, att_id, primary_defender_id, middle_defender1_id, middle_defender2_id]):
            forward_id=i
            break

                    
    assigned_roles.append(forward_id)    
    assigned_roles.append(goalie_id)

    #print "\n\n\n"
    #print "att_id="+str(att_id)+" goalie_id="+str(goalie_id)+" primary_defender_id="+str(primary_defender_id)
    # print str(assigned_roles)   
 
    return assigned_roles
    
