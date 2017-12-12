import sys
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
from config import *
from geometry import *
from numpy import inf
from math import fabs


def assign_roles_offence(state):

    # if fabs(state.ballPos.x) > HALF_FIELD_MAXX or fabs(state.ballPos.y) > HALF_FIELD_MAXY:
    #     return 0

    goal_center=Vector2D(int(-HALF_FIELD_MAXX), int(0))
    field_center=Vector2D(int(0), int(0))
    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
    bot_present = [x for x in xrange(6)] 
    assigned_roles=[] 
    goalie = 4
    assigned_roles.append(goalie)


    min_dist = inf
    attX_id = -1
    for i in bot_present:  
        if i is goalie:
            continue  
        bot_pos = Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
        dist = bot_pos.dist(ballPos)
        if dist < min_dist:
            min_dist = dist
            attX_id = i
    assigned_roles.append(attX_id)

    bot_present.remove(goalie)
    bot_present.remove(attX_id)

    min_dist = inf
    primdfX_id = -1
    for i in bot_present:    
        bot_pos = Vector2D(int(state.homePos[i].x),int(state.homePos[i].y))
        dist = bot_pos.dist(goal_center)
        if dist < min_dist:
            min_dist = dist
            primdfX_id = i
    assigned_roles.append(primdfX_id)

    bot_present.remove(primdfX_id)

    assigned_roles.extend(bot_present)
    
    print (assigned_roles) 

 
    return assigned_roles
    
