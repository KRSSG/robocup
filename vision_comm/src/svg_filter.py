import numpy as np
import pylab as pl
import sys, os

import scipy, scipy.signal as sps

import rospy
from std_msgs.msg import String

from krssg_ssl_msgs.msg import BeliefState
import time
from copy import copy
from math import *

LAST_FRAMES = 5
FPS = 20

dict_old = []
vel_dict_old = []
count1=0

# vel1=[]
# vel1_x=[]
# vel1_y=[]
# vel_rect=[]
# vel_rect_x=[]
# vel_rect_y=[]

home_kub_old=[[],[],[],[],[],[]]
away_kub_old=[[],[],[],[],[],[]]
pub = rospy.Publisher('belief_state', BeliefState, queue_size=1000)
def rectify(dict_here, vel_dict_here, dict_old, vel_dict_old):
    global count1
    try:
        dict_now = copy(dict_old[-LAST_FRAMES:])
        dict_now.append(dict_here)
        vel_dict_now = copy(vel_dict_old[-LAST_FRAMES:])
        vel_dict_now.append(vel_dict_here)
        vel_set_x = []
        vel_set_y = []
        pos_set_x = []
        pos_set_y = []
        # print("dict old:", dict_old)
        for i in xrange(len(dict_now)):
            try:
                pos_set_x.append(dict_now[i]["x"])
                vel_set_x.append(vel_dict_now[i]["x"])
                pos_set_y.append(dict_now[i]["y"])
                vel_set_y.append(vel_dict_now[i]["y"])
            except Exception as e: 
                print "here 1"
                print e
                exc_type, exc_obj, exc_tb = sys.exc_info()
                fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                print(exc_type, fname, exc_tb.tb_lineno)
                break
                # print("vs: ",vel_set)

        # print "line no: "+str(len(pos_set_x))
        print "pos set x:"+str(pos_set_x)
        rectified_set_x = sps.savgol_filter(np.array(pos_set_x),LAST_FRAMES,3)
        rectified_vel_set_x = sps.savgol_filter(np.array(vel_set_x),LAST_FRAMES,3)
        rectified_set_y = sps.savgol_filter(np.array(pos_set_y),LAST_FRAMES,3)
        rectified_vel_set_y = sps.savgol_filter(np.array(vel_set_y),LAST_FRAMES,3)
        # dict_updated = copy(dict_now[-2])

        # print("pos set x: ",pos_set_x,"rect x: ",rectified_set_x)
        # print("dict here: ",dict_here)
        # print ("rs:",rectified_set[-1][0])
        # dict_updated["x"] += rectified_set[-1][0] * (dict_now[-1]["time_now"] - dict_now[-2]["time_now"])
        # dict_updated["y"] += rectified_set[-1][1] * (dict_now[-1]["time_now"] - dict_now[-2]["time_now"])

        dict_updated = {}
        dict_updated["x"] = rectified_set_x[-1]
        dict_updated["y"] = rectified_set_y[-1]

        vel_dict_updated = {}
        vel_dict_updated["x"] = rectified_vel_set_x[-1]
        vel_dict_updated["y"] = rectified_vel_set_y[-1]

        # acc = [(rectified_set[-1][0]-rectified_set[-2][0]) * FPS, (rectified_set[-1][1]-rectified_set[-2][1]) * FPS]

        # vel_rect.append([count1,sqrt(pow(rectified_vel_set_x[-1],2)+pow(rectified_vel_set_y[-1],2))])
        # vel_rect_x.append([count1,(rectified_vel_set_x[-1])])
        # vel_rect_y.append([count1,(rectified_vel_set_y[-1])])
        count1+=1
        return dict_updated, vel_dict_updated
    except Exception as e:
        print "here 2"
        print e
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        return copy(dict_here), copy(vel_dict_here)

# arr1=[]
# arr_rect=[]

def callback(data):

    global dict_old, vel_dict_old, home_kub_old, away_kub_old, pub

    here = copy(data)
    there = copy(here)
    where = copy(here)
    rate = rospy.Rate(10)
    count2=0
    pub.publish(here);
    return;
    # print "published"

    # if (there == where):
    #     for _ in xrange(12):
    #         print"where is there"
    # return

    global count2
    print "\n\nI heard: "+str(data.ballPos.x)+" "+str(data.ballPos.y)
    dict_now = {"x": data.ballPos.x, "y":data.ballPos.y}
    vel_dict_now = {"x": data.ballVel.x, "y":data.ballVel.y}
    # arr1.append([data.ballPos.x,data.ballPos.y])
    # vel1.append([count2,sqrt(pow(data.ballVel.x,2)+pow(data.ballVel.y,2))])
    # vel1_x.append([count2,data.ballVel.x])
    # vel1_y.append([count2,data.ballVel.y])

    count2+=1

    dict_updated, vel_dict_updated = rectify(dict_now, vel_dict_now, dict_old, vel_dict_old)

    print ("ball pos org: ",data.ballPos)
    print ("ball pos rect: ",dict_updated)

    print ("ball vel org: ",data.ballVel)
    print ("ball vel rect: ",vel_dict_updated)
    # arr_rect.append([dict_updated["x"],dict_updated["y"]])

    dict_old.append(dict_updated)
    vel_dict_old.append(vel_dict_updated)


    if len(dict_old)>LAST_FRAMES:
        dict_old = dict_old[-LAST_FRAMES:]


    if len(vel_dict_old)>LAST_FRAMES:
        vel_dict_old = vel_dict_old[-LAST_FRAMES:]

    here.ballPos.x, here.ballPos.y, here.ballVel.x, here.ballVel.y = dict_updated["x"],dict_updated["y"],\
                                                                    vel_dict_updated["x"], vel_dict_updated["y"]

    for i in xrange(6):
        home_kub = {"x": data.homePos[i].x, "y":data.homePos[i].y}
        away_kub = {"x": data.awayPos[i].x, "y":data.awayPos[i].y}

        print ("i: ",i)
        print ("len of home kub old: ",len(home_kub_old))
        print ("len of away kub old: ",len(away_kub_old))
        home_dict_updated, away_dict_updated = rectify(home_kub, away_kub, home_kub_old[i], away_kub_old[i])

        home_kub_old[i].append(home_dict_updated)
        # except: 
        #         home_kub_old.append([])
        #         home_kub_old[i].append(home_dict_updated)


        away_kub_old[i].append(away_dict_updated)
        # except:
        #         away_kub_old.append([])
        #         away_kub_old[i].append(away_dict_updated)


        here.homePos[i].x = home_dict_updated["x"]
        here.homePos[i].y = home_dict_updated["y"]
        
        here.awayPos[i].x = away_dict_updated["x"]
        here.awayPos[i].y = away_dict_updated["y"]

        print "for kub "+str(i)

        print ("home pos org ",str(data.homePos[i]))
        print ("home pos rect",str(here.homePos[i]))

        print ("away pos org ",str(data.awayPos[i]))
        print ("away pos rect",str(here.awayPos[i]))


        if len(home_kub_old[i])>LAST_FRAMES:
            home_kub_old[i] = home_kub_old[i][-LAST_FRAMES:]

        if len(away_kub_old[i])>LAST_FRAMES:
            away_kub_old[i] = away_kub_old[i][-LAST_FRAMES:]
    # pub.publish(here)

    # rospy.init_node('talker', anonymous=True)
    # rate.sleep()

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('filter', anonymous=False)

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

    pl.figure()
    pl.plot(vel_rect[:,0],vel_rect[:,1],color='b')
    pl.plot(vel1[:,0],vel1[:,1],color='r')
    pl.title('vel_mod')
    # pl.show()
    pl.figure()
    pl.scatter(arr1[:,0],arr1[:,1],color='r')
    pl.scatter(arr_rect[:,0],arr_rect[:,1],color='b')
    pl.title('pos')
    # pl.show()
