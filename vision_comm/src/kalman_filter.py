import numpy as np
import pylab as pl

from pykalman.datasets import load_robot
from pykalman import KalmanFilter

import rospy
from std_msgs.msg import String

from krssg_ssl_msgs.msg import BeliefState
import time
from copy import copy
from math import *

LAST_FRAMES = 5
FPS = 30

dict_old = []
count1=0

def filter(vel_set):
    n_timesteps = len(vel_set)
    print("timesteps: ",n_timesteps)
    n_dim_state = 2
    # print("nds",n_dim_state)

    observations = np.array(vel_set)
    # x = np.zeros(n_timesteps)
    # y = np.linspace(0,50,51)
    # x=[[i] for i in x]
    # y=[[i] for i in y]

    # print observations.shape
    # Initialize the Kalman Filter
    # data = load_robot()
    kf = KalmanFilter(
        np.array([[1.2,0.01],[0.01,1.2]]), #data.transition_matrix (dim,dim)
        observations[-2:,:], #data.observation_matrix,(2,dim)
        np.eye(2), #data.initial_transition_covariance,(dim,dim)
        np.transpose(np.eye(2)), #data.initial_observation_covariance,[(0,val),(val,0)]
        np.ones((LAST_FRAMES,2))*0.001, #data.transition_offsets,(500,dim)
        np.array([1,1]), #data.observation_offset,(2)
        np.zeros(2), #data.initial_state_mean,(dim)
        np.array([[1,0.01],[0.01,1]]), #data.initial_state_covariance,(dim,dim)
        random_state=0
    )

    # Estimate mean and covariance of hidden state distribution iteratively.  This
    # is equivalent to
    #
    # (filter_state_means, filtered_state_covariance) = kf.filter(data)
    filtered_state_means = np.zeros((n_timesteps, n_dim_state))
    # print("fsm:",filtered_state_means,filtered_state_means.shape)
    filtered_state_covariances = np.zeros((n_timesteps, n_dim_state, n_dim_state))
    # print("fsc:",filtered_state_covariances,filtered_state_covariances.shape)



    # print ("dto: ",data.transition_offsets[0])
    # print ("fsm: ",filtered_state_means[0])
    # print ("fsc: ",filtered_state_covariance[0])
    # print ("dtm: ",data.transition_matrix)
    # print ("om: ",data.observation_matrix)

    TF = np.array([1.,1.])


    for t in range(n_timesteps - 1):
        if t == 0:
            filtered_state_means[t] = [0,0]
            filtered_state_covariances[t] = np.eye(2)
        filtered_state_means[t + 1], filtered_state_covariances[t + 1] = (
            kf.filter_update(
                filtered_state_means[t],
                filtered_state_covariances[t],
                observations[t + 1],
                transition_offset=TF,
            )
        )

    # draw estimates
    # pl.figure()

    # print filtered_state_means
    # lines_true = pl.plot(observations[:,0],observations[:,1], color='b')
    # lines_filt = pl.plot(filtered_state_means[:,0],filtered_state_means[:,1], color='r')
    # pl.legend((lines_true[0], lines_filt[0]), ('true', 'filtered'))
    # pl.show()

    return filtered_state_means

vel1=[]
vel1_x=[]
vel1_y=[]
vel_rect=[]
vel_rect_x=[]
vel_rect_y=[]

def rectify(dict_here):
    global count1
    try:
        dict_now = copy(dict_old[-LAST_FRAMES:])
        dict_now.append(dict_here)
        vel_set = []
        for i in xrange(len(dict_here)):
            try:
                delta_t = dict_now[-i]["time_now"] - dict_old[-(i+1)]["time_now"]
                delta_x = dict_now[-i]["x"] - dict_now[-(i+1)]["x"]
                delta_y = dict_now[-i]["y"] - dict_now[-(i+1)]["y"]
                vel_x = delta_x * FPS
                vel_y = delta_y * FPS
                vel_set.append([vel_x, vel_y])
            except Exception as e: 
                print e
                break
        # print("vs: ",vel_set)
        rectified_set = filter(vel_set)
        dict_updated = copy(dict_now[-2])
        # print ("rs:",rectified_set[-1][0])
        # dict_updated["x"] += rectified_set[-1][0] * (dict_now[-1]["time_now"] - dict_now[-2]["time_now"])
        # dict_updated["y"] += rectified_set[-1][1] * (dict_now[-1]["time_now"] - dict_now[-2]["time_now"])

        acc = [(rectified_set[-1][0]-rectified_set[-2][0]) * FPS, (rectified_set[-1][1]-rectified_set[-2][1]) * FPS]

        dict_updated["x"] += (rectified_set[-2][0] / FPS) + (0.5 * acc[0] / (pow(FPS,2)))
        dict_updated["y"] += (rectified_set[-2][1] / FPS) + (0.5 * acc[1] / (pow(FPS,2)))
        # vel_rect.append([count1,sqrt(pow(rectified_set[-1][0],2)+pow(rectified_set[-1][1],2))])
        vel_rect_x.append([count1,rectified_set[-1][0]])
        vel_rect_y.append([count1,rectified_set[-1][1]])
        count1+=1
        return dict_updated
    except Exception as e: 
                print e
                return copy(dict_here)

arr1=[]
arr_rect=[]
count2=0

def callback(data):
    global count2
    print "\n\nI heard: "+str(data.ballPos.x)+" "+str(data.ballPos.y)
    dict_now = {"x": data.ballPos.x, "y":data.ballPos.y, "time_now": time.time()}
    arr1.append([data.ballPos.x,data.ballPos.y])
    vel1.append([count2,sqrt(pow(data.ballVel.x,2)+pow(data.ballVel.y,2))])
    vel1_x.append([count2,data.ballVel.x])
    vel1_y.append([count2,data.ballVel.y])
    count2+=1

    dict_updated = rectify(dict_now)
    print dict_updated
    arr_rect.append([dict_updated["x"],dict_updated["y"]])

    dict_old.append(dict_updated)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter', anonymous=True)

    rospy.Subscriber("vision", BeliefState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    vel_rect = np.array(vel_rect)
    vel_rect_x = np.array(vel_rect_x)
    vel_rect_y = np.array(vel_rect_y)
    vel1 = np.array(vel1)
    vel1_x = np.array(vel1_x)
    vel1_y = np.array(vel1_y)
    arr_rect = np.array(arr_rect)
    arr1 = np.array(arr1)
    # print ("vel1:",(vel1))
    # print ("vel_rect:",vel_rect)
    pl.plot(vel_rect_x[:,0],vel_rect_x[:,1],color='b')
    pl.plot(vel1_x[:,0],vel1_x[:,1],color='r')
    pl.title('vel_x')
    pl.figure()
    pl.plot(vel_rect_y[:,0],vel_rect_y[:,1],color='b')
    pl.plot(vel1_y[:,0],vel1_y[:,1],color='r')
    pl.title('vel_y')
    # pl.show()
    pl.figure()
    pl.scatter(arr1[:,0],arr1[:,1],color='r')
    pl.scatter(arr_rect[:,0],arr_rect[:,1],color='b')
    pl.title('pos')
    pl.show()
