import sys
import os
import rospy

sys.path.append('./../plays_py/scripts/utils/')
from geometry import Vector2D
from config import *
from krssg_ssl_msgs.srv import path_plan
from krssg_ssl_msgs.msg import point_2d

from profiler import *
from pid import pid
from pso import PSO
from error import Error

v = None
kubid = None
expectedTraverseTime = None
pso = None
errorInfo = Error()
REPLAN = 0
FIRST_CALL = 1
homePos = None
awayPos = None

def Get_Vel(start, t, kubid_, target, homePos_, awayPos_):
    global expectedTraverseTime, REPLAN, v, errorInfo, pso, FIRST_CALL, homePos, awayPos, kubid
    REPLAN = 0
    homePos = homePos_
    awayPos = awayPos_
    kubid = kubid_

    curPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
    distance = sqrt(pow(target.x - homePos[kubid].x,2) + pow(target.y - homePos[kubid].y,2))
    if(FIRST_CALL):
        startPt = point_2d()
        startPt.x = homePos[kubid].x
        startPt.y = homePos[kubid].y
        findPath(startPt, target)
        FIRST_CALL = 0

    if distance < 1.2*BOT_BALL_THRESH:
        return [0,0,0,0]
    print("ex = ",expectedTraverseTime) 
    print("t = ",t," start = ",start)
    print("t - start = ",t-start)       
    if (t - start< expectedTraverseTime):
        print("inside ------------------------ here ")
        if v.trapezoid(t - start,curPos):
            index = v.GetExpectedPositionIndex()
            if index == -1:
                vX,vY,eX,eY = v.sendVelocity(v.getVelocity(),v.motionAngle[index],index)
                vX,vY = 0,0
                print("here-----------------0,0")

            else:
                vX,vY,eX,eY = v.sendVelocity(v.getVelocity(),v.motionAngle[index],index)

        else:
            print(t-start, expectedTraverseTime)
            if expectedTraverseTime == 'REPLAN' and False:
                REPLAN = 1
            print("Motion Not Possible")
            vX,vY,eX,eY = 0,0,0,0
            flag = 1
    else:
        print("TimeOUT, REPLANNING, time = ",t-start)
        vX,vY,eX,eY = 0,0,0,0
        errorInfo.errorIX = 0.0
        errorInfo.errorIY = 0.0
        errorInfo.lastErrorX = 0.0
        errorInfo.lastErrorY = 0.0
        startPt = point_2d()
        startPt.x = homePos[kubid].x
        startPt.y = homePos[kubid].y
        findPath(startPt,target)
        REPLAN = 1

    errorMag = sqrt(pow(eX,2) + pow(eY,2))
    print("errorMag = ",errorMag)
    print("MAX_BOT_SPEED = ",MAX_BOT_SPEED,"v.velocity = ",v.velocity)
    print("timeLap = ",t-start)
    # from numpy import inf
    if False and (shouldReplan() or \
        (errorMag > 400 and distance > 2* BOT_BALL_THRESH) or \
        REPLAN == 1):
            print("Should Replan",shouldReplan())
            print("ErrorMag",errorMag > 400 and distance > 2*BOT_BALL_THRESH)
            REPLAN = 1
            startPt = point_2d()
            startPt.x = homePos[kubid].x
            startPt.y = homePos[kubid].y
            findPath(startPt,target)
            return [0,0,0, REPLAN]  
    else:
        errorInfo.errorX = eX
        errorInfo.errorY = eY
        vX,vY = pid(vX,vY,errorInfo,pso)
        botAngle = homePos[kubid].theta
        vXBot = vX*cos(botAngle) + vY*sin(botAngle)
        vYBot = -vX*sin(botAngle) + vY*cos(botAngle)
        return [vXBot, vYBot, 0, REPLAN]            

def shouldReplan():
    return False
    global homePos,awayPos,kubid
    if v.velocity < MAX_BOT_SPEED/10:
        return False

    myPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
    obsPos = Vector2D()
    index = v.GetExpectedPositionIndex()
    for i in xrange(len(homePos)):
        if i == kubid:
            pass
        else:
            obsPos.x = int(homePos[i].x)
            obsPos.y = int(homePos[i].y)
            if v.ellipse(myPos,obsPos,v.motionAngle[index]):
                return True
    for i in xrange(len(awayPos)):
        obsPos.x = int(awayPos[i].x)
        obsPos.y = int(awayPos[i].y)
        if v.ellipse(myPos,obsPos,v.motionAngle[index]):
            return True

    return False

def findPath(startPoint,end):
    global FLAG_PATH_RECEIVED, REPLAN
    FLAG_PATH_RECEIVED = 1
    REPLAN = 1
    global v,expectedTraverseTime,kubid
    global start,target
    global pso,errorInfo
    startPt = point_2d()
    target = point_2d()
    startPt.x = startPoint.x
    startPt.y = startPoint.y
    target.x = end.x
    target.y = end.y
    print("Start Point ",startPt.x,startPt.y)
    print("Target Point",target.x,target.y)
    print("Waiting for service")
    rospy.wait_for_service('planner')

    planner = rospy.ServiceProxy('planner', path_plan)
    message = planner(startPt,target)
    path = []
    for i in xrange(len(message.path)):
        path = path + [Vector2D(int(message.path[i].x),int(message.path[i].y))]
    start = rospy.Time.now()
    start = 1.0*start.secs + 1.0*start.nsecs/pow(10,9)
    v = Velocity(path,start,startPt)
    v.updateAngle()
    expectedTraverseTime = v.getTime(v.GetPathLength())
    pso = PSO(5,20,1000,1,1,0.5)
    # pso =None
    # pso = PSO(5,20,800,0,0,0.5)
    errorInfo = Error()
    print("Path Planned")