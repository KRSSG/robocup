from PyQt4 import QtCore,QtGui
import sys
import rospy
import threading
from thread import start_new_thread
import os
import numpy as np
from math import cos, sin, atan2
from interfacePath import Ui_MainWindow
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import planner_path
from krssg_ssl_msgs.msg import point_SF
from krssg_ssl_msgs.msg import gr_Commands
import multiprocessing
MAJOR_AXIS_FACTOR = 10
MINOR_AXIS_FACTOR = 2
PI = 3.141592653589793
radius  = 10
VEL_ANGLE = 0
from utils.config import *

points_home = []
points_home_theta = []
points_opp=[]
FIELD_MAXX = HALF_FIELD_MAXX    #4000 in GrSim
FIELD_MAXY = HALF_FIELD_MAXY    #3000 in GrSim
BOT_ID = None
GUI_X = 600
GUI_Y = 400
vel_theta=0
vel_mag=0
ballPos = [0,0]
BState = None

def BS_TO_GUI(x, y):
    #GUI -> 600X400
    x1 = (x + FIELD_MAXX)*GUI_X/(2*FIELD_MAXX)
    y1 = (y + FIELD_MAXY)*GUI_Y/(2*FIELD_MAXY)


    return [x1, y1]

vrtx_0=[(200,200)]
vrtx_1=[(200,200)]
vrtx_2=[(200,200)]
vrtx_3=[(200,200)]
vrtx_4=[(200,200)]
vrtx_5=[(200,200)]

curr_vel = [10,0]
VEL_UNIT = 5
BOT_ID = 0

pub = rospy.Publisher('gui_params', point_SF)
kubs_pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

path_received=0

def debug_path(msg):
    global BOT_ID
    BOT_ID = msg.bot_id
    print("New Path Received: BOT_ID = ",BOT_ID)
    global path_received, VEL_ANGLE, vel_theta, vel_mag, vrtx_0, vrtx_1, vrtx_2, vrtx_3, vrtx_4, vrtx_5
    if(BOT_ID==0):
        vrtx_0=[]
        for v in msg.point_array:
            vrtx_0.append((BS_TO_GUI(v.x, v.y)))
    elif BOT_ID==1:
        vrtx_1=[]
        for v in msg.point_array:
            vrtx_1.append((BS_TO_GUI(v.x, v.y)))
    elif BOT_ID==2:
        vrtx_2=[]
        for v in msg.point_array:
            vrtx_2.append((BS_TO_GUI(v.x, v.y)))
    elif BOT_ID==3:
        vrtx_3=[]
        for v in msg.point_array:
            vrtx_3.append((BS_TO_GUI(v.x, v.y)))
    elif BOT_ID==4:
        vrtx_4=[]
        for v in msg.point_array:
            vrtx_4.append((BS_TO_GUI(v.x, v.y)))
    elif BOT_ID==5:
        vrtx_5=[]
        for v in msg.point_array:
            vrtx_5.append((BS_TO_GUI(v.x, v.y)))        

    path_received = 1        
    # if(vel_mag<0.01):     
    #     VEL_ANGLE = atan2(vrtx[2][1]-vrtx[0][1], vrtx[2][0]-vrtx[0][0])
    # else:
    #     VEL_ANGLE = vel_theta      

def Callback_VelProfile(msg):
    global curr_vel, vel_mag, vel_theta
    msg = msg.robot_commands
    theta = float(points_home_theta[BOT_ID])
    vel_theta = atan2(-1*msg.velnormal, msg.veltangent) + theta
    vel_mag = msg.velnormal*msg.velnormal + msg.veltangent*msg.veltangent
    curr_vel = [vel_mag, vel_theta]
    if(vel_mag<0.01):
        VEL_ANGLE = 0    
    else:
        VEL_ANGLE = vel_theta    

def Callback_BS(msg):
    global points_home, points_home_theta, points_opp, ballPos
    # print "777"
    BState = msg
    ballPos = BS_TO_GUI(msg.ballPos.x, msg.ballPos.y)
    points_home = []
    points_home_theta = []
    points_opp=[]
    # print(BS_TO_GUI(-HALF_FIELD_MAXX,0))
    for i in msg.homePos:
        points_home.append(BS_TO_GUI(i.x, i.y))
        points_home_theta.append(i.theta)  
    for i in msg.awayPos:
        points_opp.append(BS_TO_GUI(i.x, i.y))


class MainWindow(QtGui.QMainWindow, Ui_MainWindow, QtGui.QWidget):
    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)
        QtGui.QWidget.__init__(self)
        self.setupUi(self)
        self.scene = QtGui.QGraphicsScene()
        self.image = None
        self.obstacleRadius = 8

        self.graphicsView.setFixedSize(720,500)
        self.scene.setSceneRect(0, 0, 600, 400)
        self.graphicsView.setScene(self.scene)
        
        self.pen = QtGui.QPen(QtCore.Qt.green)
        self.mark_s = QtGui.QPen(QtCore.Qt.red)
        self.mark_e = QtGui.QPen(QtCore.Qt.blue)
        self.mark_ball = QtGui.QPen(QtCore.Qt.yellow)
        
        self.GoToBall.clicked.connect(self.goToBall)
        self.GoToBallFsm.clicked.connect(self.goToBallFsm)
        self.GoInTriangle.clicked.connect(self.Move_in_Triangle)
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
        self.t1 = None
        self.tri_pr1 = None
        self.tri_pr2 = None
        self.tri_pr3 = None
        self.tri_controller = None 

    
    def end_all_process(self):
        if(not self.t1==None):
            self.t1.terminate()
        if(not self.tri_pr1==None):
            self.tri_pr1.terminate()
        if(not self.tri_pr2==None):
            self.tri_pr2.terminate()
        if(not self.tri_pr3==None):
            self.tri_pr3.terminate()
        if(not self.tri_controller==None):
            self.tri_controller.terminate()    

    def goToBall(self):
        msg=point_SF()
        kubs_id = int(self.textBotId.text())
        msg.bot_id = kubs_id
        print("__here__",msg.bot_id)
        pub.publish(msg)
                    
        self.end_all_process()
        self.t1 = multiprocessing.Process(target=self.goToPoint,args=(kubs_id,))
        self.t1.start()
        # self.t1.terminate()
        #start_new_thread(self.goToPoint,())
        # start_new_thread(self.pidOperator,())
        # start_new_thread(self.velocityProfiling,())
        # return
    def goToBallFsm(self):
        global BState,kubs_pub

        self.end_all_process()
        msg=point_SF()

        # while True:
        kubs_id = int(self.textBotId.text())

        # msg.bot_id = kubs_id
        # pub.publish(msg)
        print "in goToBall",kubs_id

        if BState:
            print "belief_state Subscriber"
            kub = kubs.kubs(kubs_id,BState,kubs_pub)
            g_fsm = GoToBall.GoToBall()
            g_fsm.add_kub(kub)
            g_fsm.add_theta(theta=normalize_angle(pi+atan2(BState.ballPos.y,BState.ballPos.y-3000)))
            g_fsm.spin()
            if not (self.t1 == None):
                self.t1.terminate()
            self.t1 = multiprocessing.Process(target=g_fsm.spin)
            self.t1.start()

    def Move_in_Triangle(self):
        self.end_all_process()
        self.tri_controller = multiprocessing.Process(target=self.triangle_controller)
        self.tri_pr1 =  multiprocessing.Process(target=self.move_bot_1)
        self.tri_pr2 =  multiprocessing.Process(target=self.move_bot_2)
        self.tri_pr3 =  multiprocessing.Process(target=self.move_bot_3)
        self.tri_controller.start()
        self.tri_pr1.start()
        self.tri_pr2.start()
        self.tri_pr3.start()           

    def triangle_controller(self):
        try:
            os.system('python test_Triangle.py')
        except Exception as e:
            print("Error TRI_CONTROLLER = ",e) 

    def move_bot_1(self):
        try:
            os.system('python triangle1.py')
        except Exception as e:
            print("Error BOT_1 = ",e)    

    def move_bot_2(self):
        try:
            os.system('python triangle2.py')
        except Exception as e:
            print("Error BOT_2 = ",e)

    def move_bot_3(self):
        try:
            os.system('python triangle3.py')
        except Exception as e:
            print("Error BOT_3 = ",e)                

    def pidOperator(self):
        try:
            os.system('python ../velocity_profiling/src/pidOperator.py')
        except:
            pass


    def velocityProfiling(self):
        try:
            os.system('rosrun velocity_profiling vel_profiling')
        except:
            pass

    def path_listener(self):
        try:
            os.system('rosrun ompl_planner listener_ompl')
        except Exception as e:
            print("Error: ", e)
            pass
    def goToPoint(self,kubs_id):
        try:
            os.system('python test_GoToPoint.py '+str(kubs_id))
        except Exception as e:
            print e
            pass
    
    def drawBoundary(self):
        x1,y1 = BS_TO_GUI(HALF_FIELD_MAXX, HALF_FIELD_MAXY)
        x2,y2 = BS_TO_GUI(-HALF_FIELD_MAXX, -HALF_FIELD_MAXY)
        self.scene.addLine(x1,y1, x1, y2)
        self.scene.addLine(x1, y2, x2, y2)
        self.scene.addLine(x2,y1, x1,y1)
        self.scene.addLine(x2,y2,x2,y1)
        self.scene.addLine(x1/2,y1,x1/2,y2)
        self.scene.addEllipse((x1/2)-50,(y1/2)-50,100,100)
       
    def show_vel_vector(self):
        global curr_vel
        # print("curr_vel ", curr_vel)
        speed = curr_vel[0]
        theta = curr_vel[1]
        start_ = (points_home[0][0],points_home[0][1])
        end_ = (points_home[0][0]+VEL_UNIT*speed*cos(theta), points_home[0][1]-VEL_UNIT*speed*sin(theta))

        self.scene.addLine(start_[0],start_[1], end_[0], end_[1])

    # def sendParams(self):
    #     stepSize = float(self.stepSizeText.text())
    #     biasParam = float(self.biasParamText.text())
    #     maxIterations = float(self.maxIterationsText.text())
    #     msg=point_SF()
    #     msg.s_x=points_home[1][0]
    #     msg.s_y=points_home[1][1]
    #     msg.f_x=0
    #     msg.f_y=0
    #     msg.step_size=stepSize
    #     msg.bias_param=biasParam
    #     msg.max_iteration=maxIterations
    #     pub.publish(msg)
    #     pass        

    def updateImage(self):
       
        self.display_bots(points_home, points_opp)
        # self.show_vel_vector()

    def paintEvent(self,event):
        qp=QtGui.QPainter()
        qp.begin(self)
        qp.end()  

    def display_bots(self, points_home, points_opp):
        global ballPos
        transform = QtGui.QTransform()
       
        self.scene.clear()

        self.graphicsView.setScene(self.scene)
        brush_yellow = QtGui.QBrush(QtCore.Qt.yellow)
        brush_blue= QtGui.QBrush(QtCore.Qt.blue)
        brush_red = QtGui.QBrush(QtCore.Qt.red)
        self.drawBoundary()

        if(len(points_home)==0):
            print("SIZE OF POS_HOME = 0 ")
            return
        i = 0
        self.scene.addEllipse(ballPos[0], ballPos[1],self.obstacleRadius/2,self.obstacleRadius/2 , self.mark_ball, brush_yellow)
        for point in points_home:

            io = QtGui.QGraphicsTextItem()
            io.setDefaultTextColor(QtCore.Qt.black)
            io.setPos(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius);
            io.setPlainText(str(i));
            i=i+1
            self.scene.addEllipse(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius,2*self.obstacleRadius,2*self.obstacleRadius , self.mark_e, brush_blue)
            self.scene.addItem(io)
        i=0
        for point in points_opp:
            io = QtGui.QGraphicsTextItem()
            io.setDefaultTextColor(QtCore.Qt.black)
            io.setPos(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius);
            io.setPlainText(str(i));
            i=i+1
            self.scene.addEllipse(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius,2*self.obstacleRadius,2*self.obstacleRadius , self.mark_s, brush_red)
            self.scene.addItem(io) 
        self.draw_path()  

    def draw_path(self):
        # print("IN DRAW PATH__"*100)
        global vrtx_0, vrtx_1, vrtx_2, vrtx_3, vrtx_4, vrtx_5

        self.draw_path_one(vrtx_0)
        self.draw_path_one(vrtx_1)
        self.draw_path_one(vrtx_2)
        self.draw_path_one(vrtx_3)
        self.draw_path_one(vrtx_4)
        self.draw_path_one(vrtx_5)

    def draw_path_one(self, vrtx):
        
        path = QtGui.QPainterPath()
        path.moveTo(vrtx[0][0],vrtx[0][1])
        size_ = len(vrtx)
        division = int(size_/100)
        if(division<1):
            division = 1
        for i in vrtx[1::division]:
            path.lineTo(i[0],i[1])
           
        path.lineTo(vrtx[size_-1][0], vrtx[size_-1][1])   
        self.scene.addPath(path)

app=QtGui.QApplication(sys.argv)
w=MainWindow()
def main():
    rospy.init_node('display1', anonymous=False)
    rospy.Subscriber("/belief_state", BeliefState , Callback_BS);
    rospy.Subscriber("/grsim_data", gr_Commands , Callback_VelProfile);
    rospy.Subscriber("/path_planner_ompl", planner_path, debug_path)
    
    w.show()
    app.exec_()
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(0)

if __name__=='__main__':
    main()
