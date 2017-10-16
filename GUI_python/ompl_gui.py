from PyQt4 import QtCore,QtGui
import sys
import rospy
import numpy as np
from InterfacePath_ompl import Ui_MainWindow
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import planner_path
from krssg_ssl_msgs.msg import point_SF


points_home=[]
points_opp=[]
vrtx=[(200,200)]
pub = rospy.Publisher('gui_params', point_SF)


def debug_path(msg):
    global vrtx
    vrtx=[]
    for v in msg.point_array:
        vrtx.append(((int(v.x)),int(v.y)))
    # print("received path points, size = ",len(vrtx))        

              
    # cv2.imshow("bots", img)
    # print(" in display bots here")

def Callback(msg):
    # print(" in callback")
    global points_home
    points_home=[]
    for i in msg.homePos:
        points_home.append([(int((i.x)+3300)*3.0/32), int((i.y+2200)/10.0)])
    global points_opp    
    points_opp=[]
    for i in msg.awayPos:
        points_opp.append([(int((i.x)+3300)*3.0/32), int((i.y+2200)/10.0)])


class MainWindow(QtGui.QMainWindow, Ui_MainWindow, QtGui.QWidget):
    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)
        QtGui.QWidget.__init__(self)
        self.setupUi(self)
        self.scene = QtGui.QGraphicsScene()
        self.image = None
        self.sendData.clicked.connect(self.sendParams)
        #self.updatePath.clicked.connect(self.update_path)
        self.refresh.clicked.connect(self.hide_all)
        self.obstacleRadius = 10
        self.graphicsView.setFixedSize(650,450)
        self.scene.setSceneRect(0, 0, 600, 400)
        self.graphicsView.setScene(self.scene)
        self.hide_all()
        self.pen = QtGui.QPen(QtCore.Qt.green)
        self.mark_s = QtGui.QPen(QtCore.Qt.red)
        self.mark_e = QtGui.QPen(QtCore.Qt.blue)

        # self.videoFrame=ImageWidget()
        # self.setCentralWidget(self.videoFrame)
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(30)
        # self.capture = cv2.VideoCapture(0)
    def hide_all(self):
        # self.setCentralWidget(self)
        pass

    def sendParams(self):
        print("in send_params")
        stepSize = float(self.stepSizeText.text())
        biasParam = float(self.biasParamText.text())
        maxIterations = float(self.maxIterationsText.text())
        msg=point_SF()
        msg.s_x=points_home[1][0]
        msg.s_y=points_home[1][1]
        msg.f_x=0
        msg.f_y=0
        # print(" here step_size ",stepSize)
        msg.step_size=stepSize
        msg.bias_param=biasParam
        msg.max_iteration=maxIterations
        pub.publish(msg)
        pass        

    def updateImage(self):
       
        self.display_bots(points_home, points_opp)

    def paintEvent(self,event):
        # print(" in paint event")
        qp=QtGui.QPainter()
        qp.begin(self)
        qp.end()  

    def display_bots(self, points_home, points_opp):
        global vrtx
        self.scene.clear()
        self.graphicsView.setScene(self.scene)
        brush= QtGui.QBrush(QtCore.Qt.SolidPattern)
        for point in points_home:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_e, brush)
        for point in points_opp:
            self.scene.addEllipse(point[0], point[1],self.obstacleRadius,self.obstacleRadius , self.mark_s, brush) 
        self.draw_path(vrtx)  

    def draw_path(self, vrtx):
    # print("in draw_path")
        path = QtGui.QPainterPath()
        # path_points = []
        # with open("../../../path.dat") as f:
        #     content = f.readlines()
        #     content = content[0].strip()
        #     print(content)
        #     path_points.append((int(float(content[0])/800.0*600), int(float(content[1])/500.0*400)))

        # vrtx = path_points
        # print("no of points = ", len(vrtx))  
          
        path.moveTo(vrtx[0][0],vrtx[0][1])
        max_x=0
        max_y=0
        min_x=999
        min_y=999
        for i in vrtx[1::5]:
            path.lineTo(i[0],i[1])
            if(i[0]>max_x):
                max_x=i[0]
            if(i[0]<min_x):
                min_x=i[0]

            if(i[1]>max_y):
                max_y=i[1] 
            if(i[1]<min_y):
                min_y=i[1]       
        
        self.scene.addPath(path)
        # print(" Path added to scene now ",max_x, max_y," min = ",min_x, min_y)

app=QtGui.QApplication(sys.argv)
w=MainWindow()
def main():
    rospy.init_node('display', anonymous=True)
    rospy.Subscriber("/belief_state", BeliefState , Callback);
    rospy.Subscriber("/path_planner_ompl", planner_path, debug_path)

    w.show()
    app.exec_()

if __name__=='__main__':
    main()