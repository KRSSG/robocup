import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

import rospy
from std_msgs.msg import String

from krssg_ssl_msgs.msg import BeliefState, SSL_DetectionFrame, SSL_DetectionBall, SSL_DetectionRobot
import time
from copy import copy
from math import *
import time


dict_old = []
count1=0

def makefilter():
    global kf
    kf = KalmanFilter(dim_x=4, dim_z=2)
    dt = 0.003
    kf.F = np.array([[1, dt, 0,  0],
                     [0,  1, 0,  0],
                     [0,  0, 1, dt],
                     [0,  0, 0,  1]])
    kf.u = 0
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])
    kf.R = np.eye(2) * 0.1**2
    q = Q_discrete_white_noise(dim=2, dt=dt, var=1.5**2)
    kf.Q = block_diag(q, q)
    kf.x = np.array([0, 0, 0, 0])
    kf.P = np.eye(4) * 200

def updateFilter(dt):
    global kf
    kf.F = np.array([[1, dt, 0,  0],
                     [0,  1, 0,  0],
                     [0,  0, 1, dt],
                     [0,  0, 0,  1]])
vel_rect=[]

def filter(dict_here):
    global count1, kf
    z = np.array([dict_here["x"], dict_here["y"]])
    try:
        updateFilter(dict_old[-1]["time_now"] - dict_old[-2]["time_now"])
    except Exception:
        pass
    kf.predict()
    kf.update(z)
    pos = [kf.x[0], kf.x[2]]
    vel = [kf.x[1], kf.x[3]]
    count1+=1
    return pos, vel

arr1=[]
arr_rect=[]
count2=0

def callback(data):
    global count2
    try:
        # i=0
        # for i in range(len(data.robots_blue)):
        #     if data.robots_blue[i].robot_id == 0:
        #         break
        # print "\n\nI heard: "+str(data.robots_blue[i].x)+" "+str(data.robots_blue[i].y)
        # dict_now = {"x": data.robots_blue[i].x, "y":data.robots_blue[i].y, "time_now": time.time()}
        # arr1.append([data.robots_blue[i].x,data.robots_blue[i].y])
        # if count2==0:
        #     kf.x = np.array([data.robots_blue[i].x, 0, data.robots_blue[i].y, 0])
        print "\n\nI heard: "+str(data.balls[0].x)+" "+str(data.balls[0].y)
        dict_now = {"x": data.balls[0].x, "y":data.balls[0].y, "time_now": time.time()}
        arr1.append([data.balls[0].x,data.balls[0].y])
        if count2==0:
            kf.x = np.array([data.balls[0].x, 0, data.balls[0].y, 0])
        count2+=1
        dict_old.append(dict_now)
        start = time.time()
        pos, vel = filter(dict_now)
        end = time.time()
        print(end-start)
        print count2
        arr_rect.append(pos)
        vel_rect.append(vel)
    except Exception as e:
        print e
        pass

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter', anonymous=True)

    rospy.Subscriber("vision", SSL_DetectionFrame, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    makefilter()
    listener()
    vel_rect = np.array(vel_rect)
    arr_rect = np.array(arr_rect)
    arr1 = np.array(arr1)
    # plotting the results
    plt.figure()
    plt.plot(np.arange(0, len(vel_rect)),vel_rect[:,0], color = 'g', label = 'filter')
    plt.xlabel('number of measurments')
    plt.ylabel('x-velocity')
    plt.title('vel_x')
    plt.legend()
    plt.figure()
    plt.plot(np.arange(0, len(vel_rect)),vel_rect[:,1], color = 'g', label = 'filter')
    plt.xlabel('number of measurments')
    plt.ylabel('y-velocity')
    plt.title('vel_y')
    plt.legend()
    plt.figure()
    plt.plot(np.arange(0, len(arr_rect)),arr_rect[:,0], ls='--', color = 'k', label = 'filter')
    plt.plot(np.arange(0, len(arr1)),arr1[:,0], color = 'r', label = 'vision')
    plt.xlabel('number of measurments')
    plt.ylabel('x')
    plt.title('pos_x')
    plt.legend()
    plt.figure()
    plt.plot(np.arange(0, len(arr_rect)),arr_rect[:,1], ls='--', color = 'k', label = 'filter')
    plt.plot(np.arange(0, len(arr1)),arr1[:,1], color= 'r', label = 'vision')
    plt.xlabel('number of measurments')
    plt.ylabel('y')
    plt.title('pos_y')
    plt.legend()
    plt.show()
