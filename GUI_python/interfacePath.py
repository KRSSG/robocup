# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sampledraw.ui'
#
# Created: Thu Aug 17 20:42:15 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1140, 670)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.graphicsView = QtGui.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(0, 0, 650, 410))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.stepSizeText = QtGui.QLineEdit(self.centralwidget)
        self.stepSizeText.setGeometry(QtCore.QRect(990, 40, 91, 27))
        self.stepSizeText.setObjectName(_fromUtf8("stepSizeText"))
        self.stepSize = QtGui.QLabel(self.centralwidget)
        self.stepSize.setGeometry(QtCore.QRect(810, 50, 66, 17))
        self.stepSize.setObjectName(_fromUtf8("stepSize"))
        self.maxIterationsText = QtGui.QLineEdit(self.centralwidget)
        self.maxIterationsText.setGeometry(QtCore.QRect(900, 80, 81, 27))
        self.maxIterationsText.setObjectName(_fromUtf8("maxIterationsText"))
        self.biasParamText = QtGui.QLineEdit(self.centralwidget)
        self.biasParamText.setGeometry(QtCore.QRect(890, 130, 81, 27))
        self.biasParamText.setObjectName(_fromUtf8("biasParamText"))
        self.maxIterations = QtGui.QLabel(self.centralwidget)
        self.maxIterations.setGeometry(QtCore.QRect(790, 90, 91, 20))
        self.maxIterations.setObjectName(_fromUtf8("maxIterations"))
        self.biasParam = QtGui.QLabel(self.centralwidget)
        self.biasParam.setGeometry(QtCore.QRect(800, 130, 71, 20))
        self.biasParam.setObjectName(_fromUtf8("biasParam"))
        self.sendData = QtGui.QPushButton(self.centralwidget)
        self.sendData.setGeometry(QtCore.QRect(880, 210, 98, 27))
        self.sendData.setObjectName(_fromUtf8("sendData"))
        self.refresh = QtGui.QPushButton(self.centralwidget)
        self.refresh.setGeometry(QtCore.QRect(680, 420, 98, 27))
        self.refresh.setObjectName(_fromUtf8("refresh"))
        self.startPoint = QtGui.QPushButton(self.centralwidget)
        self.startPoint.setGeometry(QtCore.QRect(70, 480, 181, 27))
        self.startPoint.setObjectName(_fromUtf8("startPoint"))
        self.endPoint = QtGui.QPushButton(self.centralwidget)
        self.endPoint.setGeometry(QtCore.QRect(280, 480, 211, 27))
        self.endPoint.setObjectName(_fromUtf8("endPoint"))
        self.startPointText = QtGui.QLineEdit(self.centralwidget)
        self.startPointText.setGeometry(QtCore.QRect(90, 520, 113, 27))
        self.startPointText.setObjectName(_fromUtf8("startPointText"))
        self.endPointText = QtGui.QLineEdit(self.centralwidget)
        self.endPointText.setGeometry(QtCore.QRect(320, 520, 113, 27))
        self.endPointText.setObjectName(_fromUtf8("endPointText"))
        self.updatePath = QtGui.QPushButton(self.centralwidget)
        self.updatePath.setGeometry(QtCore.QRect(680, 310, 98, 27))
        self.updatePath.setObjectName(_fromUtf8("updatePath"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1126, 25))
        self.menubar.setNativeMenuBar(False)
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuDIFFERENT_RRT = QtGui.QMenu(self.menubar)
        self.menuDIFFERENT_RRT.setObjectName(_fromUtf8("menuDIFFERENT_RRT"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.actionRRT_CONNECT = QtGui.QAction(MainWindow)
        self.actionRRT_CONNECT.setObjectName(_fromUtf8("actionRRT_CONNECT"))
        self.actionRRT_X = QtGui.QAction(MainWindow)
        self.actionRRT_X.setObjectName(_fromUtf8("actionRRT_X"))
        self.menuDIFFERENT_RRT.addAction(self.actionRRT_CONNECT)
        self.menuDIFFERENT_RRT.addAction(self.actionRRT_X)
        self.menubar.addAction(self.menuDIFFERENT_RRT.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.biasParam.show)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.biasParamText.show)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.maxIterations.show)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.maxIterationsText.show)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.stepSize.show)
        QtCore.QObject.connect(self.actionRRT_CONNECT, QtCore.SIGNAL(_fromUtf8("triggered()")), self.stepSizeText.show)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.stepSize.setText(_translate("MainWindow", "StepSize", None))
        self.maxIterations.setText(_translate("MainWindow", "MaxIterations", None))
        self.biasParam.setText(_translate("MainWindow", "BiasParam", None))
        self.sendData.setText(_translate("MainWindow", "sendData", None))
        self.refresh.setText(_translate("MainWindow", "refresh", None))
        self.startPoint.setText(_translate("MainWindow", "choosen start point", None))
        self.endPoint.setText(_translate("MainWindow", "choosen end point", None))
        self.updatePath.setText(_translate("MainWindow", "updatePath", None))
        self.menuDIFFERENT_RRT.setTitle(_translate("MainWindow", "DIFFERENT_RRT", None))
        self.actionRRT_CONNECT.setText(_translate("MainWindow", "RRT_CONNECT", None))
        self.actionRRT_X.setText(_translate("MainWindow", "RRT_X", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
