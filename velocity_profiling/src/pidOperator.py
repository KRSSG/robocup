import sys
import rospy
from pid import pid
from pso import PSO
from error import Error
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import pid_message
from math import cos, sin, pow

global lastTime
global pub

pso = PSO(15,20,1000,1,1,0.5)
errorInfo = Error()

# homePos = []
# homeBotTheta = []
# def CallbackBS(msg):
#     # print("BeliefState OK")
#     for id in msg.homePos:
#         homePos.append([id.x,id.y])
#         homeBotTheta.append(id.theta)

def CallbackPID(msg):
    global pub
    global lastTime
    global pso
    # print("PID Callback OK")
    velX = msg.velX*1000
    velY = msg.velY*1000
    flag = msg.flag
    if flag:
        print("Replanned")
        pso = PSO(15,20,1000,1,1,0.5)

    errorInfo.errorX = msg.errorX
    errorInfo.errorY = msg.errorY
    # print("INitial",velX,velY)
    vX,vY = pid(velX,velY,errorInfo,pso)
    # vX,vY = velX,velY
    # print("Changed",vX,vY)
    botAngle = msg.botAngle
    # print("BotAngle ", botAngle)
    vXBot = vX*cos(botAngle) + vY*sin(botAngle)
    vYBot = -vX*sin(botAngle) + vY*cos(botAngle)
    # print("Velocity ",vXBot,vYBot)


    command_msgs = gr_Robot_Command()
    final_msgs = gr_Commands()

    command_msgs.id          = msg.id
    command_msgs.wheelsspeed = 0
    command_msgs.veltangent  = vXBot/1000
    command_msgs.velnormal   = vYBot/1000
    command_msgs.velangular  = 0
    command_msgs.kickspeedx  = 0
    command_msgs.kickspeedz  = 0
    command_msgs.spinner     = False

    if(msg.velX == msg.velY and msg.velX==0):
        command_msgs.velnormal = command_msgs.veltangent = 0
    # t = rospy.get_rostime()
    # currTime = t.secs + t.nsecs/pow(10,9)
    # diffT = float(currTime - lastTime)
    # command_msgs.nextExpectedX = vXBot*diffT + homePos[0][0];
    # command_msgs.nextExpectedY = vYBot*diffT + homePos[0][1]; 
    
    # final_msgs.timestamp      = ros::Time::now().toSec()
    final_msgs.isteamyellow   = False
    final_msgs.robot_commands = command_msgs
    # lastTime = currTime
    pub.publish(final_msgs)


def main():
    global pub
    global lastTime
    # print("Starting pid operator")
    rospy.init_node('pidOperator', anonymous=False)
    # t = rospy.get_rostime()
    # lastTime = t.secs + t.nsecs/pow(10,9)
    # rospy.Subscriber("/belief_state", BeliefState , CallbackBS);
    # rospy.Subscriber("/grsim_data", gr_Commands , Callback_VelProfile);
    rospy.Subscriber("/pid", pid_message, CallbackPID)
    pub = rospy.Publisher("/grsim_data",gr_Commands, queue_size=1000)
    rospy.spin()


if __name__=='__main__':
     main()