import sys
sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages/')
import rospy
import serial
from pylab import *
import math
import time 
sys.path.append('../../')
frame = 1
ti = time.time()            
# sys.path.append('/home/aif/ROBOCUP_SSL_WS/devel/lib/python2.7/dist-packages/krssg_ssl_msgs/msg/')
from krssg_ssl_msgs.msg import gr_Commands
ser = None

# # ser = serial.Serial('/dev/ttyUSB1',bytesize=32)
# ser = serial.Serial('/dev/ttyUSB1')
file = ''
try :
    try:
        global ser, file
        ser = serial.Serial('/dev/ttyUSB0',115200)
        file = '/dev/ttyUSB0'
    except:
        global ser
        ser = serial.Serial('/dev/ttyUSB1',115200)
        file = '/dev/ttyUSB1'
    # ser = serial.Serial('/dev/ttyUSB1',bytesize=32)
    # print(ser)

except :
    print("Couldn't open the port\n")


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
FACTOR_T = 30
FACTOR_N = 30
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
            v_4_wheel[i] = 126 + ((v_4_wheel[i]-max_vel_wheel)*126) / max_vel_wheel
        else :
            v_4_wheel[i] = 255 + (v_4_wheel[i]*129) / max_vel_wheel
    return v_4_wheel

def gr_Commands_CB(msg):
    global buf
    #print("Command received for bot_id : ",msg.robot_commands.id)
    vel_xyw = [None]*3
    vel_xyw[0] = int(msg.robot_commands.velnormal * FACTOR_T)
    vel_xyw[1] = -1*int(msg.robot_commands.veltangent * FACTOR_N)
    vel_xyw[2] =int(msg.robot_commands.velangular * FACTOR_W)
    v_4_wheel = vel_convert(vel_xyw)
    global frame,ti
    # print(frame, ti,"  ", time.time())
    # print("Frame Rate  ",frame*1.0/(time.time()-ti))
    frame += 1
    start = 1 + msg.robot_commands.id*5
    for i in range(4):
        buf[i+start] = int(v_4_wheel[i])

    if msg.robot_commands.spinner and msg.robot_commands.kickspeedx :
        buf[start+4] = 3
    elif msg.robot_commands.spinner :
        buf[start+4] = 2
    elif msg.robot_commands.kickspeedx :
        buf[start+4] = 1
    else:
        buf[start+4] = 0

    # print(msg.robot_commands.spinner, msg.robot_commands.kickspeedx,"dribbler test")

    #ser.flush()
    #buf = bytearray(buf)

    buff = ''
    
    for i in xrange(len(buf)):
        # if i%5 is 0 and i:
        #   buf[i] = 3
        if buf[i] > 255:
            print(buf[i])
            buf[i] = 80
        #print("f#ck off ",i)
        buff += chr(int(buf[i])%256)
    #print(buff)
    #print("2 fucked ",buf)
    # time.sleep(0.5)
    # statinfo = stat(file)
  
    # print(statinfo.st_size,file)
    #ser.flushInput()

    print(buf)
    ser.write(buff)
    # print(,"data return ")
    #print(size_buf_file(file),file)
    # print("Data sent :")
    


rospy.init_node('bot_comm_nrf',anonymous=False)
rospy.Subscriber('/grsim_data',gr_Commands,gr_Commands_CB)
# time.sleep(0.5)
rospy.spin()
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