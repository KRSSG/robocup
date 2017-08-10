
import sys
from interfacePath import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets


class MyFirstGuiProgram(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        #these will be given by pathplanner
        self.obstacles=[(5,8) , (100,100) , (10 ,50)] 
        self.pathPoints=[(100,50) , ( -100 ,-200), (-100,120) , (100,100)]
        self.obstacleRadius= 50

        
        QtWidgets.QMainWindow.__init__(self, parent=parent)
        self.setupUi(self)
        self.scene = QtWidgets.QGraphicsScene()
        self.graphicsView.setScene(self.scene)
        self.pen = QtGui.QPen(QtCore.Qt.green)
        self.submit.clicked.connect(self.sendParams)
        
    def paintEvent(self, e):
        self.drawObstacles()
        self.drawPath()
        

    def drawObstacles(self ):
        brush= QtGui.QBrush(QtCore.Qt.SolidPattern)
        for i in self.obstacles:
            self.scene.addEllipse(i[0],i[1],self.obstacleRadius,self.obstacleRadius , self.pen, brush)

    def drawPath(self):
        path = QtGui.QPainterPath()
        path.moveTo(self.pathPoints[0][0],self.pathPoints[0][1])
        for i in self.pathPoints:
            path.lineTo(i[0],i[1])
        self.scene.addPath(path)

    def sendParams(self):

        stepSize = float(self.stepSizeText.text())
        biasParam = float(self.biasParamText.text())
        maxIter = float(self.maxIterationsText.text())
        print("THESE DATA HAVE TO BE SEND TO PATHPLANNER stepsize {} , maxiter {} , biasparam {} " .format(stepSize,maxIter , biasParam))



if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = MyFirstGuiProgram()
    w.show()
    sys.exit(app.exec_())