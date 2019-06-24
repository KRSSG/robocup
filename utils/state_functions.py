from geometry import Vector2D
from config import *
from math_functions import *


def ball_in_front_of_bot(kub):
    theta1 = kub.get_pos().theta
    theta2 = math.atan2(kub.state.ballPos.y - kub.get_pos().y,
                        kub.state.ballPos.x - kub.get_pos().x)
    return vicinity_theta(theta1, theta2, thresh=0.25) and vicinity_points(kub.get_pos(), kub.state.ballPos, thresh=BOT_RADIUS * 4)

def bot_in_front_of_ball(kub, thresh = BOT_RADIUS*1.5):
    return vicinity_points(kub.get_pos(), kub.state.ballPos, thresh)

def kub_has_ball(state, kub_id, is_opponent=False):
    if is_opponent:
        theta1 = state.awayPos[kub_id].theta
        theta2 = math.atan2(
            state.ballPos.y - state.awayPos[kub_id].y, state.ballPos.x - state.awayPos[kub_id].x)
        return vicinity_theta(theta1, theta2, thresh=0.25) and vicinity_points(state.awayPos[kub_id], state.ballPos, thresh=BOT_RADIUS * 1.5)
    else:
        theta1 = state.homePos[kub_id].theta
        theta2 = math.atan2(
            state.ballPos.y - state.homePos[kub_id].y, state.ballPos.x - state.homePos[kub_id].x)
        return vicinity_theta(theta1, theta2, thresh=0.25) and vicinity_points(state.homePos[kub_id], state.ballPos, thresh=BOT_RADIUS * 1.5)

##
## Check if ball is moving towards our goal based on magnitude of velocity 
## and approximate aim of ball
## 
##
def ball_moving_towards_our_goal(state):
    ballPos = Vector2D(state.ballPos.x, state.ballPos.y)
    ballvel = Vector2D(state.ballvel.x, state.ballvel.y)
    if ballvel.absSq(ballvel) > 0.1:
        ball_movement = Line(ballPos, ballPos + ballvel)
        ptA = Vector2D(-HALF_FIELD_MAXX, DBOX_HEIGHT)
        ptB = Vector2D(-HALF_FIELD_MAXX, -DBOX_HEIGHT)
        defend_line = Line(point1=ptA,point2=ptB)
        opponent_aim = defend_line.intersection_with_line(ball_movement)
        if opponent_aim.y > -DBOX_HEIGHT and opponent_aim.y < DBOX_HEIGHT:
            return True
    return False


##
## Check if opponent team has ball based on distance and angle of opponent bot
## @return     Id of opponent bot if opponent team has ball else None
##
def opponent_bot_with_ball(state):
    ballPos = Vector2D(state.ballPos.x, state.ballPos.y)
    closest_bot = None
    distance = 999999
    for opp_id, awayPos in enumerate(state.awayPos):
        opp_pos = Vector2D(awayPos.x, awayPos.y)
        dist = opp_pos.dist(ballPos)
        if dist < distance:
            distance = dist
            closest_bot = opp_id
    if closest_bot is not None:
        if kub_has_ball(state, closest_bot, is_opponent = True):
            return closest_bot
    return None


def ball_in_our_goalie_region(state):
    pass

##
## @return    Id of opponent bot closest to position
##
def closest_opponent(state, position):
    p = position
    closest_bot, closest_dist = float("inf")
    for opp_id, awayPos in enumerate(state.awayPos):
        opp_pos = Vector2D(awayPos.x, awayPos.y)
        dist = opp_pos.dist(p)
        if dist < closest_dist:
            closest_dist = dist
            closest_bot = opp_id

    return closest_bot

def our_bot_closest_to_ball(state):
    distance_from_ball = 99999999
    our_bot_closest_to_ball = 0
    for i in range(len(state.homePos)):
        dist = math.sqrt(pow((state.homePos[i].x - state.ballPos.x),2) + pow((state.homePos[i].y - state.ballPos.y) , 2))
        if dist < distance_from_ball :
                distance_from_ball = dist
                our_bot_closest_to_ball = i

    return our_bot_closest_to_ball

def opp_bot_closest_to_ball(state):
    distance_from_ball = 99999999
    opp_bot_closest_to_ball = 0
    for i in range(len(state.awayPos)):
        dist = math.sqrt(pow((state.awayPos[i].x - state.ballPos.x),2) + pow((state.awayPos[i].y - state.ballPos.y) , 2))
        if dist < distance_from_ball :
                distance_from_ball = dist
                opp_bot_closest_to_ball = i

    return opp_bot_closest_to_ball

def ball_in_our_half(state):
    if state.ballPos.x <= 0 :
        return True
    else :
        return False



