import sys
import rospy
import serial, os
import math
import time 
frame = 1
ti = time.time()      

import socket

HOST = '0.0.0.0'
PORT = 44444
BUFFER_SIZE = 8
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST,PORT))
from krssg_ssl_msgs.msg import gr_Commands

Team_ID = 127  # Team Blue
theta = [30.0,150.0,225.0,315.0] # Bot angles
bot_radius = 0.087
bot_wheel_radius = 0.025
max_vel_wheel = 5000.0 # in rpm
# global buf
buf = [None]*32
for i in range(32):
    buf[i] = 0
buf[0] = Team_ID
FACTOR_T = 40
FACTOR_N = 40
FACTOR_W = 90
v_4_wheel = [None, None, None, None]
ch_buff = ''
from os import stat

add = ('192.168.1.3',44444)


def vel_convert(vel_3_wheel):
    vx = vel_3_wheel[0]
    vy = vel_3_wheel[1]
    vw = -vel_3_wheel[2]

    print("vx",vx,"vy",1,"vw",vw)
    for i in range(4):
        v_4_wheel[i] = ((bot_radius*vw) - (vx*math.sin(theta[i]*math.pi/180.0)) + (vy*math.cos(theta[i]*math.pi/180.0)))/(bot_wheel_radius * math.pi)
    for i in range(4):
        if v_4_wheel[i] > 0 :
            v_4_wheel[i] = int(126 + ((v_4_wheel[i]-max_vel_wheel)*126) / max_vel_wheel)
        else :
            v_4_wheel[i] = int(256 - (v_4_wheel[i]*(129-256)) / max_vel_wheel)
    return v_4_wheel

def gr_Commands_CB(msg):
    print("...")
    global buf
    #print("Command received for bot_id : ",msg.robot_commands.id)
    vel_xyw = [None]*3
    vel_xyw[0] = int(msg.robot_commands.velnormal * FACTOR_T)
    vel_xyw[1] = -1*int(msg.robot_commands.veltangent * FACTOR_N)
    vel_xyw[2] =int(msg.robot_commands.velangular * FACTOR_W)
    # vel_xyw[2] = 0
    # vel_xyw = [0, 300 ,0]
    v_4_wheel = vel_convert(vel_xyw)
    print("Velocities ",vel_xyw)
    global frame,ti
    # print(frame, ti,"  ", time.time())
    # print("Frame Rate  ",frame*1.0/(time.time()-ti))
    frame += 1
    start = 1 + msg.robot_commands.id*5
    for i in range(4):
        buf[i+start] = int(v_4_wheel[i])
        if buf[i+start] == 255:
            buf[i+start] = 0

    temp = buf[9]
    buf[9] = buf[8]
    buf[8] = buf[7]
    buf[7] = buf[6]
    buf[6] = temp
    print("Data: ", buf)
    if msg.robot_commands.spinner and msg.robot_commands.kickspeedx :
        buf[start+4] = 3
    elif msg.robot_commands.spinner :
        buf[start+4] = 2
    elif msg.robot_commands.kickspeedx :
        buf[start+4] = 1
    else:
        buf[start+4] = 0


    buff = ''
    
    for i in xrange(len(buf)):
        # if i%5 is 0 and i:
        #   buf[i] = 3
        if buf[i] > 255:
            # print(buf[i])
            buf[i] = 0
        #print("f#ck off ",i)
        buff += chr(int(buf[i])%256)

    print(buf)
    data = s.recvfrom(BUFFER_SIZE)
    if data:
        print('Client to server: ', data)
        s.sendto(buff, data[1])
        print("sent msg : ", buff)
    print("done")



    # print("waiting to receivg")
    # try:
    #     s.sendto(buff, add)
    # except:
    #     pass
    # data = s.recvfrom(BUFFER_SIZE)
    # if data:
    #     print('Client to server: ', data)
    #     print("data sent")
    



rospy.init_node('bot_comm_wifi',anonymous=False)
rospy.Subscriber('/grsim_data',gr_Commands,gr_Commands_CB)
# time.sleep(0.5)
rospy.spin()
