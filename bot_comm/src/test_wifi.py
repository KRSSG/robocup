import sys
import rospy
import serial, os
import math
import time 
# sys.path.append('../../')
frame = 1
ti = time.time()      

import socket

HOST = '0.0.0.0'
PORT = 44444
BUFFER_SIZE = 8
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST,PORT))
# # sys.path.append('/home/aif/ROBOCUP_SSL_WS/devel/lib/python2.7/dist-packages/krssg_ssl_msgs/msg/')
# from krssg_ssl_msgs.msg import gr_Commands
# ser = None
# prev_time = time.time()
# now_time = time.time()
# # ser = serial.Serial('/dev/ttyUSB1',bytesize=32)
# ser = serial.Serial('/dev/ttyUSB1')
# file = ''
# try :
#     try:
#         # global ser, file
#         ser = serial.Serial('/dev/ttyUSB0',115200)
#         file = '/dev/ttyUSB0'
#     except:
#         # global ser
#         ser = serial.Serial('/dev/ttyUSB1',115200)
#         file = '/dev/ttyUSB1'
#     # ser = serial.Serial('/dev/ttyUSB1',bytesize=32)
#     # print(ser)

# except :
#     print("Couldn't open the port\n")


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


def vel_convert(vel_3_wheel):
    vx = vel_3_wheel[0]
    vy = vel_3_wheel[1]
    vw = -vel_3_wheel[2]

    print("vx",vx,"vy",vy,"vw",vw)
    for i in range(4):
        v_4_wheel[i] = ((bot_radius*vw) - (vx*math.sin(theta[i]*math.pi/180.0)) + (vy*math.cos(theta[i]*math.pi/180.0)))/(bot_wheel_radius * math.pi)
    for i in range(4):
        if v_4_wheel[i] > 0 :
            v_4_wheel[i] = int(126 + ((v_4_wheel[i]-max_vel_wheel)*126.0) / max_vel_wheel)
        else :
            v_4_wheel[i] = int(256 - (v_4_wheel[i]*(129.0-256.0)) / max_vel_wheel)
    return v_4_wheel

def gr_Commands_CB(msg):
    global buf
    #print("Command received for bot_id : ",msg.robot_commands.id)
    # vel_xyw = [None]*3
    # vel_xyw[0] = int(msg.robot_commands.velnormal * FACTOR_T)
    # vel_xyw[1] = -1*int(msg.robot_commands.veltangent * FACTOR_N)
    # vel_xyw[2] =int(msg.robot_commands.velangular * FACTOR_W)
    # v_4_wheel = vel_convert(vel_xyw)
    # global frame,ti
    # # print(frame, ti,"  ", time.time())
    # # print("Frame Rate  ",frame*1.0/(time.time()-ti))
    # frame += 1
    # start = 1 + msg.robot_commands.id*5
    # for i in range(4):
    #     buf[i+start] = int(v_4_wheel[i])

    # if msg.robot_commands.spinner and msg.robot_commands.kickspeedx :
    #     buf[start+4] = 3
    # elif msg.robot_commands.spinner :
    #     buf[start+4] = 2
    # elif msg.robot_commands.kickspeedx :
    #     buf[start+4] = 1
    # else:
    #     buf[start+4] = 0

    # print(msg.robot_commands.spinner, msg.robot_commands.kickspeedx,"dribbler test")

    #ser.flush()
    #buf = bytearray(buf)

    buff = ''
    
    for i in xrange(1,32):
        buf[i] = 0


    buf[5], buf[6], buf[7], buf[8] = vel_convert([-30, 0,0,])
    buf[5], buf[6], buf[7], buf[8] = [0,0,0,0]
    for i in xrange(5,9):
        buf[i] = int(buf[i])
    for i in xrange(5,9):
        buf[i] = int(buf[i])

    for i in xrange(32):
        buff += chr(int(buf[i])%256)

    print(buf)
    print("waiting to receivg")
    data = s.recvfrom(BUFFER_SIZE)
    if data:
        print('Client to server: ', data)
        s.sendto(buff, data[1])
        print("sent msg : ", buff)

    # for i in range(0,6):    
    #     # pass
    # 	buf[5*i+1],   buf[5*i+2],  buf[5*i+3],  buf[5*i+4] = vel_convert([0,-60,0])
    # # buf[1],   buf[2],  buf[3],  buf[4] = vel_convert([0,-50,0])
    # now_time = time.time()
    # # print now_time- prev_time
    # if(now_time- prev_time>1):
    #     prev_time = now_time
    #     buf[31] = 1
    #     print("SENDING_RESET"*20)

    # for i in xrange(11,15):
    #     buf[i] = int(buf[i])

    # for i in xrange(1,5):
    #     buf[i] = int(buf[i])
    # for i in xrange(32):
    #     #print("f#ck off ",i)
    #     buff += chr(int(buf[i])%256)






        
    #print(buff)
    #print("2 fucked ",buf)
    # time.sleep(0.5)
    # statinfo = stat(file)
  
    # print(statinfo.st_size,file)
    #ser.flushInput()

    # print(buf)
    # ser.write(buff)
    # print(,"data return ")
    #print(size_buf_file(file),file)
    # print("Data sent :")
    


# rospy.init_node('bot_comm_nrf',anonymous=False)
# rospy.Subscriber('/grsim_data',gr_Commands,gr_Commands_CB)
# time.sleep(0.5)
while(1):
    # os.system("sleep 0.01s")
    #time.sleep(0.1);
    gr_Commands_CB("")
    print "gfsgh"
# rospy.spin()
# msg=gr_Commands()
# msg.robot_commands.id=0
# msg.robot_commands.veltangent=0
# msg.robot_commands.velnormal=50
# msg.robot_commands.velangular=0
# for i in range(32*69):
#   time.sleep(1)
#   msg.robot_commands.veltangent=(i%3)*20
#   msg.robot_commands.velnormal=0
#   msg.robot_commands.velangular=0
#   print("itera   ", i)
#   gr_Commands_CB(msg)