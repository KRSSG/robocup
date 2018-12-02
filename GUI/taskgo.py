from PyQt4 import QtCore,QtGui
import sys
import rospy
import threading
from thread import start_new_thread
import os
import numpy as np
from math import cos, sin, atan2, sqrt
# from interfacePath import Ui_MainWindow
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import planner_path

from task import Ui_MainWindow
# from taskcsgo import GUI_link
import sys
from utils.config import *
from utils.functions import *


points_home = []
points_home_theta = []
points_opp=[]
points_opp_theta = []
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
    y1 = (-y + FIELD_MAXY)*GUI_Y/(2*FIELD_MAXY)


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


class MainWindow(QtGui.QMainWindow, Ui_MainWindow, QtGui.QWidget):
    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)
        # GUI_link.__init__(self)
        QtGui.QWidget.__init__(self)
        self.setupUi(self)


        self.scene = QtGui.QGraphicsScene()
        self.image = None
        self.obstacleRadius = 8

        self.graphicsView.setFixedSize(720,500)
        self.scene.setSceneRect(0, 0, 600, 400)
        self.graphicsView.setScene(self.scene)
        

        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)


        print "WORKS LIKE BUTTER"
        #initializations
        
        self.pen = QtGui.QPen(QtCore.Qt.green)
        self.mark_s = QtGui.QPen(QtCore.Qt.red)
        self.mark_e = QtGui.QPen(QtCore.Qt.blue)
        self.mark_ball = QtGui.QPen(QtCore.Qt.yellow)
        self.boundary = QtGui.QPen(QtCore.Qt.white)

        # comboBoxes in play testing
        self.pathOptions.activated[str].connect(self.path_choice)
        self.initialPositionOptions.activated[str].connect(self.initialPosition_choice)
        self.teamOptions.activated[str].connect(self.team_choice)
        self.goalieOptions.activated[str].connect(self.goalie_choice)
        self.playOptions.activated[str].connect(self.play_choice)
        # comboBoxes in skill testing
        self.skillOptions.activated[str].connect(self.skill_choice)
        self.botOptions.activated[str].connect(self.bot_choice)
        #start button in skill testing
        self.skillStartButton.clicked.connect(self.skill_start)
        self.playStartButton.clicked.connect(self.play_start)
        self.startPlaySelectorButton.clicked.connect(self.play_selector_start)
        #exitButton
        self.exitButton.clicked.connect(self.close_application)

        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
        self.run_play_selector = None


    def end_all_process(self):    
        if (not self.run_play_selector==None):
            self.run_play_selector.terminate()


        

    def drawBoundary(self, pen, brush):
        # print "Drawing"
        x1,y1 = BS_TO_GUI(HALF_FIELD_MAXX, HALF_FIELD_MAXY)
        x2,y2 = BS_TO_GUI(-HALF_FIELD_MAXX, -HALF_FIELD_MAXY)
        self.scene.addLine(x1,y1, x1, y2, pen)
        self.scene.addLine(x1, y2, x2, y2, pen)
        self.scene.addLine(x2,y1, x1,y1, pen)
        self.scene.addLine(x2,y2,x2,y1, pen)
        self.scene.addLine(x1/2,y1,x1/2,y2, pen)
        self.scene.addEllipse((x1/2)-50,(y2/2)-50,100,100, pen)
        path = QtGui.QPainterPath()
        alongX = DBOX_WIDTH*GUI_X/(2*HALF_FIELD_MAXX)
        alongY = (DBOX_HEIGHT - OUR_GOAL_MAXY)*GUI_Y/(2*HALF_FIELD_MAXY)
        sweepDegrees = 90
        # Draw DBOX
        # Opp side
        opp_vertexA = BS_TO_GUI(HALF_FIELD_MAXX,OUR_DBOX_MAXY)
        opp_vertexB = BS_TO_GUI(-OUR_DBOX_X,OUR_DBOX_MAXY)
        opp_vertexC = BS_TO_GUI(-OUR_DBOX_X,OUR_DBOX_MINY)
        opp_vertexD = BS_TO_GUI(HALF_FIELD_MAXX,OUR_DBOX_MINY)
        self.scene.addLine(opp_vertexA[0],opp_vertexA[1],opp_vertexB[0],opp_vertexB[1],pen)
        self.scene.addLine(opp_vertexB[0],opp_vertexB[1],opp_vertexC[0],opp_vertexC[1],pen)
        self.scene.addLine(opp_vertexC[0],opp_vertexC[1],opp_vertexD[0],opp_vertexD[1],pen)

        # Our Side
        our_vertexA = BS_TO_GUI(-HALF_FIELD_MAXX,OUR_DBOX_MAXY)
        our_vertexB = BS_TO_GUI(OUR_DBOX_X,OUR_DBOX_MAXY)
        our_vertexC = BS_TO_GUI(OUR_DBOX_X,OUR_DBOX_MINY)
        our_vertexD = BS_TO_GUI(-HALF_FIELD_MAXX,OUR_DBOX_MINY)
        self.scene.addLine(our_vertexA[0],our_vertexA[1],our_vertexB[0],our_vertexB[1],pen)
        self.scene.addLine(our_vertexB[0],our_vertexB[1],our_vertexC[0],our_vertexC[1],pen)
        self.scene.addLine(our_vertexC[0],our_vertexC[1],our_vertexD[0],our_vertexD[1],pen)

        # goal post on opp side
        goal_depth = GOAL_DEPTH*GUI_X/(2*HALF_FIELD_MAXX)
        goal_width = OUR_GOAL_WIDTH*GUI_Y/(2*HALF_FIELD_MAXY)
        pointA = BS_TO_GUI(HALF_FIELD_MAXX, OUR_GOAL_MAXY)
        path.addRect(pointA[0], pointA[1], goal_depth, goal_width)
        self.scene.addPath(path, pen)

        # goal post on our side
        goal_depth = GOAL_DEPTH*GUI_X/(2*HALF_FIELD_MAXX)
        goal_width = OUR_GOAL_WIDTH*GUI_Y/(2*HALF_FIELD_MAXY)
        pointA = BS_TO_GUI(-HALF_FIELD_MAXX - GOAL_DEPTH, OUR_GOAL_MAXY)
        path.addRect(pointA[0], pointA[1], goal_depth, goal_width)
        self.scene.addPath(path, pen)


    def updateImage(self):
       
        self.display_bots(points_home, points_opp)

    def display_bots(self, points_home, points_opp):
        global ballPos
        transform = QtGui.QTransform()
       
        self.scene.clear()
        self.draw_path()

        self.graphicsView.setScene(self.scene)
        brush_yellow = QtGui.QBrush(QtCore.Qt.yellow)
        brush_blue= QtGui.QBrush(QtCore.Qt.blue)
        brush_red = QtGui.QBrush(QtCore.Qt.red)
        brush_white = QtGui.QBrush(QtCore.Qt.white)
        brush_darkred = QtGui.QBrush(QtCore.Qt.darkRed)
        blue_pen = QtGui.QPen(QtCore.Qt.blue)
        yellow_pen =QtGui.QPen(QtCore.Qt.yellow)
        darkred_pen = QtGui.QPen(QtCore.Qt.darkRed)
        self.drawBoundary(self.boundary, brush_white)

        if(len(points_home)==0):
            print("SIZE OF POS_HOME = 0 ")
            return
        i = 0
        # show ball
        ball_dimension = 6
        self.scene.addEllipse(ballPos[0]- ball_dimension/2, ballPos[1] - ball_dimension/2,ball_dimension,ball_dimension , darkred_pen, brush_darkred)

        # show home bots
        for point in points_home:

            io = QtGui.QGraphicsTextItem()
            io.setDefaultTextColor(QtCore.Qt.white)
            io.setPos(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius*1.5);
            io.setPlainText(str(i));
            path =QtGui.QPainterPath()
            sweepDegrees = 270
            angle = radian_2_deg(points_home_theta[i])
            vertex = [point[0]- self.obstacleRadius, point[1] - self.obstacleRadius]
            path.arcMoveTo(vertex[0], vertex[1],2*self.obstacleRadius,2*self.obstacleRadius, 315 + angle)
            path.arcTo(vertex[0], vertex[1], 2*self.obstacleRadius, 2*self.obstacleRadius, 45 + angle, sweepDegrees)
            self.scene.addPath(path, blue_pen, brush_blue)
            self.scene.addItem(io)
            i=i+1
        i=0
        # show opp bots
        for point in points_opp:
            io = QtGui.QGraphicsTextItem()
            io.setDefaultTextColor(QtCore.Qt.black)
            io.setPos(point[0]-self.obstacleRadius, point[1]-self.obstacleRadius*1.5);
            io.setPlainText(str(i));
            path =QtGui.QPainterPath()
            sweepDegrees = 270
            angle = radian_2_deg(points_opp_theta[i])
            vertex = [point[0]- self.obstacleRadius, point[1] - self.obstacleRadius]
            path.arcMoveTo(vertex[0], vertex[1],2*self.obstacleRadius,2*self.obstacleRadius, 315 + angle)
            path.arcTo(vertex[0], vertex[1], 2*self.obstacleRadius, 2*self.obstacleRadius, 45 + angle, sweepDegrees)
            self.scene.addPath(path, yellow_pen, brush_yellow)
            i=i+1
            self.scene.addItem(io) 
    def play_selector_start(self):

        self.end_all_process()
        self.run_play_selector = threading.Thread(target=self.play_selector_triggered)
        self.run_play_selector.start()
        # pass   
    def skill_start(self):
        self.skill_triggered()
    def play_start(self):
        self.play_triggered()

    def path_choice(self,text):
        self.path = text 
    def initialPosition_choice(self,text):
        self.initialPosition = text
    def team_choice(self,text):
        self.team = text
    def goalie_choice(self,text):
        self.goalie = text
    def play_choice(self,text):
        self.play = text
    def skill_choice(self,text):
        self.skillchoice = text
    def bot_choice(self,text):
        self.bot = text
    
    def draw_path(self):
        # print("IN DRAW PATH__"*100)
        global vrtx_0, vrtx_1, vrtx_2, vrtx_3, vrtx_4, vrtx_5
        try:
            self.draw_path_one(vrtx_0)
            self.draw_path_one(vrtx_1)
            self.draw_path_one(vrtx_2)
            self.draw_path_one(vrtx_3)
            self.draw_path_one(vrtx_4)
            self.draw_path_one(vrtx_5)
        except:
            pass

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
        self.scene.addPath(path, self.pen)

    def close_application(self):
        choice = QtGui.QMessageBox.question(self, 'RUN YOU FOOLS!', "Exit Application?", QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        if choice == QtGui.QMessageBox.Yes:
            print("Exited Application")
            sys.exit()
        else:
            pass

app=QtGui.QApplication(sys.argv)
w=MainWindow()

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

# def Callback_VelProfile(msg):
#     global curr_vel, vel_mag, vel_theta
#     msg = msg.robot_commands
#     theta = float(points_home_theta[BOT_ID])
#     vel_theta = atan2(-1*msg.velnormal, msg.veltangent) + theta
#     vel_mag = msg.velnormal*msg.velnormal + msg.veltangent*msg.veltangent
#     curr_vel = [vel_mag, vel_theta]
#     if(vel_mag<0.01):
#         VEL_ANGLE = 0    
#     else:
#         VEL_ANGLE = vel_theta


def Callback_BS(msg):
    global points_home, points_home_theta, points_opp, ballPos, points_opp_theta
    # print "777"
    BState = msg
    ballPos = BS_TO_GUI(msg.ballPos.x, msg.ballPos.y)
    points_home = []
    points_home_theta = []
    points_opp=[]
    points_opp_theta = []
    # print(BS_TO_GUI(-HALF_FIELD_MAXX,0))
    for i in msg.homePos:
        # print "{} {}".format(i.x,i.y)
        points_home.append(BS_TO_GUI(i.x, i.y))
        points_home_theta.append(i.theta)  
    for i in msg.awayPos:
        points_opp.append(BS_TO_GUI(i.x, i.y))
        points_opp_theta.append(i.theta)



def main():
    rospy.init_node('display1', anonymous=False)
    rospy.Subscriber("/belief_state", BeliefState , Callback_BS);
    # rospy.Subscriber("/grsim_data", gr_Commands , Callback_VelProfile);
    rospy.Subscriber("/path_planner_ompl", planner_path, debug_path)
    
    w.show()
    app.exec_()
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(0)

if __name__=='__main__':
    main()
