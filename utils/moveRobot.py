# Integrated old run.py and __GoToPoint__.py
# This file handles integration of PID, Velocity Profiling and Path Planning.

from kubs import kubs, cmd_node
from velocity.run_w import *
from velocity.profiler import *
from velocity.pid import pid
from velocity.pso import PSO
from velocity.error import Error
import rospy,sys
from krssg_ssl_msgs.msg import point_2d
from utils.geometry import Vector2D
from utils.config import *
from krssg_ssl_msgs.srv import *
from utils.functions import *
import math


class moveRobot():
    def __init__(self, _kub, _target_point, _final_angle, _start_time, _DISTANCE_THRES, _avoid_ball=False):
        rospy.wait_for_service('bsServer',)
        self.getState = rospy.ServiceProxy('bsServer',bsServer)

        rospy.wait_for_service('planner')
        self.planner = rospy.ServiceProxy('planner', path_plan)

        self.prev_state = None
        try:
            self.prev_state = self.getState(self.prev_state).stateB
        except rospy.ServiceException, e:
            print("Error ", e)

        self.kub = _kub
        self.target_point = Vector2D(_target_point.x, _target_point.y)
        self.final_angle = _final_angle
        self.start_time = _start_time
        self.DISTANCE_THRES = _DISTANCE_THRES
        self.vx_end = 0
        self.vy_end = 0
        self.avoid_ball = _avoid_ball
        self.replan_required = False
        self.pso = None

        self.path_profiling()

    def path_profiling(self):
        self.findPath(self.prev_state.homePos[self.kub.kubs_id], self.target_point)
        self.reset()

    def reset(self):
        start_time = rospy.Time.now()
        self.start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)

    def get_vel(self, cur_time):

        kubid = self.kub.kubs_id
        homePos = self.kub.state.homePos
        awayPos = self.kub.state.awayPos

        curPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
        distance = self.target_point.dist(curPos)

        if distance < self.DISTANCE_THRES:
            rospy.loginfo("Nearby destiantion. Approximate to reaching destination.")
            return [0,0,0]

        if self.expectedTraverseTime == 'REPLAN':
            rospy.loginfo("Replan due to expectedTraverseTime.")
            self.replan_required = True
            ############### Change 0 to suitable vw
            return [0, 0, 0]


        if cur_time - self.start_time < self.expectedTraverseTime:
            if self.vel_profiler.trapezoid(cur_time - self.start_time, curPos):
                index = self.vel_profiler.GetExpectedPositionIndex()
                # index==-1 means travelled distance is greater than path lenght
                # hence velocity in vel_profile is set to 0
                vX, vY, eX, eY = self.vel_profiler.sendVelocity(self.vel_profiler.getVelocity(),
                                    self.vel_profiler.motionAngle[index], index)
            else:
                vX, vY, eX, eY = 0, 0, 0, 0
        else:
            # timeOut
            rospy.loginfo("Replan due to timeout.")
            self.errorInfo = Error()
            vX, vY, eX, eY = 0, 0, 0, 0
            self.replan_required = True

        errorMag = sqrt(pow(eX,2) + pow(eY,2))
        
        # Check obstacles
        if self.shouldReplan():
            rospy.loginfo("Replan due to obstacles.")
            self.replan_required = True
            return [0, 0, 0]
        elif errorMag > 350 and distance > 1.5*BOT_BALL_THRESH:
            rospy.loginfo("Replan due to high error from expected position.")
            self.replan_required = True
            return [0, 0, 0]
        else:
            self.errorInfo.errorX = eX
            self.errorInfo.errorY = eY
            vX, vY = pid(vX, vY, self.errorInfo, self.pso)
            botAngle = self.kub.state.homePos[kubid].theta
            vX_kub = vX*cos(botAngle) + vY*sin(botAngle)
            vY_kub = -vX*sin(botAngle) + vY*cos(botAngle)
            return [vX_kub, vY_kub, 0]



    def findPath(self, _start_point, _dest_point):
        start_point = point_2d()
        dest_point = point_2d()

        start_point.x, start_point.y = _start_point.x, _start_point.y
        dest_point.x, dest_point.y = _dest_point.x, _dest_point.y

        message = self.planner(self.kub.kubs_id, start_point, dest_point, self.avoid_ball)
        path = []
        for i in xrange(len(message.path)):
            path = path + [Vector2D(int(message.path[i].x),int(message.path[i].y))]

        cur_time = rospy.Time.now()
        cur_time = 1.0*cur_time.secs + 1.0*cur_time.nsecs/pow(10,9)
        self.vel_profiler = Velocity(path, cur_time, start_point)
        self.vel_profiler.updateAngle()
        self.expectedTraverseTime = self.vel_profiler.getTime(self.vel_profiler.GetPathLength())
        self.errorInfo = Error()

    def shouldReplan(self):
        cur_state = self.kub.state
        kubid = self.kub.kubs_id
        homePos = cur_state.homePos
        awayPos = cur_state.awayPos
        if self.vel_profiler.velocity < 10:
            return False

        myPos = Vector2D(int(homePos[kubid].x),int(homePos[kubid].y))
        obsPos = Vector2D()
        index = self.vel_profiler.GetExpectedPositionIndex()
        for i in xrange(len(homePos)):
            if i == self.kub.kubs_id:
                pass
            else:
                obsPos.x = int(homePos[i].x)
                obsPos.y = int(homePos[i].y)
                if self.vel_profiler.ellipse(myPos,obsPos,self.vel_profiler.motionAngle[index]):
                    return True
        for i in xrange(len(awayPos)):
            obsPos.x = int(awayPos[i].x)
            obsPos.y = int(awayPos[i].y)
            if self.vel_profiler.ellipse(myPos,obsPos,self.vel_profiler.motionAngle[index]):
                return True

        return False

    def execute(self, _should_replan=True):
        # _should_replan is used to check if replanning is allowed or not

        FLAG_turn = True ## temporary
        FLAG_move = False
        REPLANNED = False
        while not (FLAG_move and FLAG_turn):
            try:
                self.kub.state = self.getState(self.prev_state).stateB
            except rospy.ServiceException, e:
                print("Error ", e)

            self.prev_state = self.kub.state

            t = rospy.Time.now()
            t = t.secs + 1.0*t.nsecs/pow(10,9)

            vx, vy, vw = self.get_vel(t)

            velocity_magnitude = Vector2D(vx,vy).abs(Vector2D(vx,vy))
            
            if velocity_magnitude > MAX_BOT_SPEED:
                angle_movement = math.atan2(vy,vx)
                vy = MAX_BOT_SPEED*math.sin(angle_movement)
                vx = MAX_BOT_SPEED*math.cos(angle_movement)

            # vw = Get_Omega(self.kub.kubs_id, self.final_angle, self.kub.state.homePos)
            
            if not vw:
                vw = 0

            if self.replan_required:
                self.path_profiling()
                if not _should_replan:
                    FLAG_turn = True
                    FLAG_move = True
                    vx, vy = 0, 0
                self.replan_required = False
            else:
                pass

            if not vx and not vy:
                vx,vy = self.vx_end, self.vy_end
            else:
                self.vx_end, self.vy_end = vx,vy


            if abs(normalize_angle(self.kub.state.homePos[self.kub.kubs_id].theta-self.final_angle))<ROTATION_FACTOR:
                self.kub.turn(0)
                FLAG_turn = True

            else:
                self.kub.turn(vw)

            if dist(self.kub.state.homePos[self.kub.kubs_id], self.target_point)<self.DISTANCE_THRES :
                self.kub.move(0,0)
                FLAG_move = True
            else:
                self.kub.move(vx, vy)

            self.kub.execute()

            yield self.kub, self.target_point

        
        self.kub.execute()

        yield self.kub, self.target_point
