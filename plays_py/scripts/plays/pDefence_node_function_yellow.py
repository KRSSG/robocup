import sys
sys.path.append('../../../skills_py/scripts/skills')
sys.path.append('../../../plays_py/scripts/utils/')
sys.path.insert(0, '../../../navigation_py/scripts/navigation/src')
sys.path.insert(0, '../../../navigation_py/scripts/navigation')
from config import *
from geometry import *
from numpy import inf
from math import *



def angle_at_vextex(P1,P2,P3):
    a     = P2.dist(P3)
    b     = P1.dist(P3)
    c     = P2.dist(P1)
    theta = acos((b**2 + c**2 - a**2)/(2*b*c))
    return theta

#returns whether a pt lies inside a triangle

kam_vale_bot = [-1,-1,-1,-1,-1,-1]
assigned_roles = []
# print(assigned_roles)
def point_in_traingle(P1, P2, P3, P):
    def area(p1,p2,p3):
        return abs(p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y) + p3.x*(p1.y-p2.y))/2.0

    a  = area(P1,P2,P3)
    a1 = area(P,P2,P3)
    a2 = area(P1,P,P3)
    a3 = area(P1,P2,P)

    return a == a1+a2+a3

def ball_velocity_direction(state):
    ballVel = Vector2D(int(state.ballVel.x), int(state.ballVel.y))
    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))

    Upper_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MAXY / 2 + BOT_BALL_THRESH))
    Lower_limit = Vector2D(-HALF_FIELD_MAXX, int(OUR_GOAL_MINY / 2 - BOT_BALL_THRESH))

    alpha_1 = atan((Upper_limit.y - ballPos.y)/ (ballPos.x - Upper_limit.x)) 
    alpha_2 = atan((Lower_limit.y - ballPos.y)/ (ballPos.x - Lower_limit.x))

    if ballVel.x < 0 or 1:
        try:
            ball_vel_angle = atan(state.ballVel.y / state.ballVel.x)
        except:
            if state.ballVel.y < 0:
                ball_vel_angle = -pi/2
            elif state.ballVel.y >0:
                ball_vel_angle = pi/2
            else:
                ball_vel_angle = 0
    else:
        return False, ball_vel_angle
    #deciding the direction of the ball's velocity
    if ballVel.y > 0:
        if(ball_vel_angle > alpha_2 and ball_vel_angle < alpha_1):
            return True,ball_vel_angle
        else :
            return False,ball_vel_angle
    else:
        if(abs(ball_vel_angle) > abs(alpha_1) and abs(ball_vel_angle) < abs(alpha_2)):
            return True,ball_vel_angle
        else:
            return False,ball_vel_angle

def threat_with_ball(state):
    threat = -1
    away_in_our_side_with_ball = []
    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
    for away_botID in xrange(len(state.awayPos)):
        if state.awayPos[away_botID].x < 0 and ballPos.dist(state.awayPos[away_botID]) <= DRIBBLER_BALL_THRESH :
            threat = away_botID

    if threat is -1:
        min_dist = inf
        ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
        for away_bot in xrange(len(state.awayPos)):
            dist = ballPos.dist(Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y)))
            if dist < min_dist:
                min_dist = dist
                threat = away_bot


    #print("bhosdiwala opponent with ball : ",threat)
    return threat

threat_bot_g = -1
threatPos_g = Vector2D(0,0)
#returns the botID of the primary threat i.e, the bot which is likely to recieve the pass else if the ball's velocity is less than certain threshold function returns -1


def primary_threat(state):
    thresh = 100000.0
    threat = -1
    away_in_our_side = []
    dist_list =[]
    angle_diff_list = []
    angle_diff_list_ball = []
    open_angle_list = []
    away_in_our_side = []
    home_in_our_side = []
    #calculate botID of opponent on our goalie side
    global threat_bot_g
    global threatPos_g
    threat_bot_g = threat_with_ball(state)
    # print("there attacker",threat_bot_g)
    ball_dir_in_goal, ball_vel_angle = ball_velocity_direction(state)

    opp_goalie = -1
    goal = Vector2D(HALF_FIELD_MAXX,0)

    mindist1 = inf
    for id in xrange(len(state.awayPos)):
        dist = goal.dist(Vector2D(int(state.awayPos[id].x),int(state.awayPos[id].y)))
        if (state.awayPos[id].x > HALF_FIELD_MAXX - DBOX_WIDTH) and (fabs(state.awayPos[id].y) < 700 and opp_goalie==-1):
            opp_goalie =id
            mindist1=dist
        if (state.awayPos[id].x > HALF_FIELD_MAXX - DBOX_WIDTH) and (fabs(state.awayPos[id].y) < 700 and opp_goalie!=-1):
            if dist < mindist1:
                mindist1 = dist
                opp_goalie = id

    # print("opp goalie ",opp_goalie)
    # mindist2 = 999999
    # if opp_goalie==-1:
    #     for id in xrange(len(state.awayPos)):
    #         dist = goal.dist(Vector2D(int(awayPos[id].x),int(awayPos[id].y)))
    #         if dist<750 and dist < mindist2:
    #             mindist2 = dist
    #             opp_goalie = id

   

    Upper_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MAXY / 1.2 + BOT_BALL_THRESH))
    Lower_limit = Vector2D(int(-HALF_FIELD_MAXX), int(OUR_GOAL_MINY / 1.2 - BOT_BALL_THRESH))

    #opponents closer to our goalie side not the primary_threat
    for away_botID in xrange(len(state.awayDetected)):
        if state.awayPos[away_botID].x < 0 :
            away_in_our_side.append(away_botID)
    for home_botID in xrange(len(state.homeDetected)):
        if state.homePos[home_botID].x <0:
            home_in_our_side.append(home_botID)
  
    secondary_threat_id = -1

    ballPos = Vector2D(int(state.ballPos.x), int(state.ballPos.y))
    if threat_bot_g is not -1:
        threatPos_g = Vector2D(int(state.awayPos[threat_bot_g].x), int(state.awayPos[threat_bot_g].y))
        

        for away_bot in  xrange(len(state.awayPos)):
            if away_bot is threat_bot_g or away_bot is opp_goalie:
                open_angle_list.append([away_bot,0])
                angle_diff_list.append([away_bot,0])
                dist_list.append([away_bot,0])
                angle_diff_list_ball.append([away_bot,0])
            else:
                awayBotPos = Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y))
                ########################################################################################################################################
                OpenAngle = abs(angle_at_vextex(awayBotPos, Upper_limit, Lower_limit))
                # print "open agnle"
                for bots in away_in_our_side:
                    if bots is not away_bot:
                        if point_in_traingle(awayBotPos, Upper_limit, Lower_limit, Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y))):
                            temp_theta = 2*asin(BOT_RADIUS*1.0/awayBotPos.dist(Vector2D(int(state.awayPos[bots].x), int(state.awayPos[bots].y))))
                            OpenAngle -= temp_theta
                for bots in home_in_our_side:
                    if point_in_traingle(state.awayPos[away_botID],Upper_limit,Lower_limit,state.homePos[bots]):
                        temp_theta = 2*asin(BOT_RADIUS*1.0/awayBotPos.dist(Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y))))
                        OpenAngle -= temp_theta
                if OpenAngle<0:
                    open_angle_list.append([away_bot,0])
                else:
                    open_angle_list.append([away_bot,exp(-0.2*pi/OpenAngle)])
                    # print(away_bot,exp(-0.2*pi/OpenAngle),OpenAngle)
                ###########################################################################################################################################
                Point_trial = Vector2D()
                angle_diff = fabs(Point_trial.normalizeAngle(fabs(state.awayPos[threat_bot_g].theta)+ fabs(state.awayPos[away_bot].theta)-pi))
                angle_diff_list.append([away_bot, exp(-1.0*angle_diff/ pi)])
                # print(away_bot,angle_diff,exp(-1.0*pow(1*angle_diff/(pi),1)))

                dist = threatPos_g.dist(Vector2D(int(state.awayPos[away_bot].x), int(state.awayPos[away_bot].y)))
                dist_list.append([away_bot, exp(-1*dist/ HALF_FIELD_MAXY)])
                # print(away_bot,dist,exp(-1*dist/ HALF_FIELD_MAXY))

                angle_diff_ball =fabs(Point_trial.normalizeAngle(fabs(ball_vel_angle)+fabs(state.awayPos[away_bot].theta) - pi))
                angle_diff_list_ball.append([away_bot, exp(-1.0*angle_diff_ball/ pi)])
            # print("ball vel")
            # print(away_bot,angle_diff_ball, exp(-1.0*angle_diff_ball/ pi))

        # print(angle_diff_list_ball)
        # print(open_angle_list)
        scores = []
        # print("angle diff",angle_diff_list)
        # print("angle diff bhosdiwala",angle_diff_list_ball)
        # print("angle diff rahul chutiya",dist_list)
        weights = [0.9, 0.4, 0.9, 0.8]    
        #print("gaandu opponent    ", threat_bot_g)    
            #bot-bot-anglediff,dist,ball-botanglediff,openangle
        

        for i in xrange(len(state.awayPos)):
            # if i is threat_bot_g:
            #     continue
            sc = angle_diff_list[i][1]*weights[0] + dist_list[i][1]*weights[1] + angle_diff_list_ball[i][1]*weights[2] + open_angle_list[i][1]*weights[3]
            # print(sc)
            # print(angle_diff_list[i][0])
            scores.append([angle_diff_list[i][0], sc])
        scores.sort(key = lambda x:x[1],reverse = True)
        # print "Threat ========="
        # print(scores)
        return scores[0][0],scores[1][0]
    else:
        return -1,-1
def assign_roles(state):
    # global assigned_roles
    assigned_roles=[]
    # print(assigned_roles)
    p_threat, s_threat = primary_threat(state)
    # global kam_vale_bot
    # print(kam_vale_bot)
    # if -1 in kam_vale_bot or 1:
    print("kam vale bot")
    print(kam_vale_bot)
    print("assigned bots")
    print(assigned_roles)
    bots_present = [x for x in xrange(6)]

    

    p_cooker = -1
    min_dist = inf
    assigned_roles.append(4)  #goalie
    bots_present.remove(4)
    # kam_vale_bot[4]=4
    #threatPos_g_g = Vector2D(int(state.awayPos[threat_bot].x), int(state.awayPos[threat_bot].y))
    global threatPos_g
    global threat_bot_g

    for home_bot in bots_present:
        dist = threatPos_g.dist(Vector2D(int(state.homePos[home_bot].x), int(state.homePos[home_bot].y)))
        if dist < min_dist:
            min_dist = dist
            p_cooker = home_bot

    

    #print("gandu gar mra",threat_bot_g)
    assigned_roles.append([p_cooker, threat_bot_g])
    bots_present.remove(p_cooker)
    # kam_vale_bot[p_cooker]=p_cooker
    print("in node function:")
    print("threats",p_threat,s_threat)
    #s_threat = secondary_threat(state, p_threat)
    #print("_-_-_-",p_threat,s_threat,"_-_-_-")
    Centre = Vector2D(int(-HALF_FIELD_MAXX), 0)
    bot_dist_list = []
    for bots in bots_present:
        bot_dist = Centre.dist(Vector2D(int(state.homePos[bots].x), int(state.homePos[bots].y)))
        bot_dist_list.append([bots, bot_dist])
    
    bot_dist_list.sort(key = lambda x:x[1])
    wall = [bot_dist_list[0][0], bot_dist_list[1][0]]
    wall.sort()
    # print("bots before assigning wall")
    # print(bots_present)
    bots_present.remove(bot_dist_list[0][0])
    bots_present.remove(bot_dist_list[1][0])
    
    # print("bots after assigning wall")
    # print(bots_present)
    p_threat_pos = Vector2D(int(state.awayPos[p_threat].x), int(state.awayPos[p_threat].y))
    s_threat_pos = Vector2D(int(state.awayPos[s_threat].x), int(state.awayPos[s_threat].y))

    distp_list =[]
    dists_list = []

    for home_bot in bots_present:
        distp = p_threat_pos.dist(Vector2D(int(state.homePos[home_bot].x), int(state.homePos[home_bot].y)))
        distp_list.append([home_bot, distp])
        dists = s_threat_pos.dist(Vector2D(int(state.homePos[home_bot].x), int(state.homePos[home_bot].y)))
        dists_list.append([home_bot, dists])
    
    distp_list.sort(key=lambda x:x[1])
    dists_list.sort(key=lambda x:x[1])

    #print("ps", [distp_list[0][0],p_threat] , [dists_list[1][0],s_threat])
    if distp_list[0][0] is not dists_list[0][0]:
        assigned_roles.extend([[distp_list[0][0], p_threat], [dists_list[0][0], s_threat]])
        bots_present.remove(distp_list[0][0])
        bots_present.remove(dists_list[0][0])
        # kam_vale_bot[distp_list[0][0]]=distp_list[0][0]
        # kam_vale_bot[dists_list[1][0]]=dists_list[1][0]
        #print(bots_present)
    else:
        assigned_roles.extend([[distp_list[0][0], p_threat], [dists_list[1][0], s_threat]])
        bots_present.remove(distp_list[0][0])
        #print("2",bots_present)
        bots_present.remove(dists_list[1][0])
        # kam_vale_bot[distp_list[0][0]]=distp_list[0][0]
        # kam_vale_bot[dists_list[1][0]]=dists_list[1][0]
        #print(bots_present)

    # kam_vale_bot[wall[0]]=wall[0]
    # kam_vale_bot[wall[1]]=wall[1]
    assigned_roles.extend(wall)
    #bot used to mark, bot to mark

    # print("assign roles G PC M0 M1 D0 D1")
    # print(assigned_roles)

    #assigned_roles.extend([])

    #Tmark

    # if fabs(state.ballPos.x) > HALF_FIELD_MAXX or fabs(state.ballPos.y) > HALF_FIELD_MAXY:
    #     return 0
    assigned_roles = [4,[0,threat_bot_g],[1,p_threat],[2,s_threat],3,5]
    file = open("node_debuggin.txt","a")
    print(assigned_roles)
    file.write("\n@@@@@@@@@@@@@@@@\n")
    file.write("marker " + str(assigned_roles[2]) +" "+ str(p_threat))
    file.write("\nmarker "+str(assigned_roles[3]) + " " +str(s_threat))
    file.write("\nwall " +str(assigned_roles[4]))
    file.write("\nwall " + str(assigned_roles[5]))

    return assigned_roles

   