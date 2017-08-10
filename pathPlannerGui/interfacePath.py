# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sampledraw.ui'
#
# Created: Wed Aug  9 23:24:41 2017
#      by: PyQt5 UI code generator 5.2.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(823, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(0, 0, 600, 400))
        self.graphicsView.setObjectName("graphicsView")
        self.stepSizeText = QtWidgets.QLineEdit(self.centralwidget)
        self.stepSizeText.setGeometry(QtCore.QRect(702, 50, 91, 27))
        self.stepSizeText.setObjectName("stepSizeText")
        self.stepSize = QtWidgets.QLabel(self.centralwidget)
        self.stepSize.setGeometry(QtCore.QRect(620, 50, 66, 17))
        self.stepSize.setObjectName("stepSize")
        self.maxIterationsText = QtWidgets.QLineEdit(self.centralwidget)
        self.maxIterationsText.setGeometry(QtCore.QRect(710, 90, 81, 27))
        self.maxIterationsText.setObjectName("maxIterationsText")
        self.biasParamText = QtWidgets.QLineEdit(self.centralwidget)
        self.biasParamText.setGeometry(QtCore.QRect(710, 140, 81, 27))
        self.biasParamText.setObjectName("biasParamText")
        self.maxIterations = QtWidgets.QLabel(self.centralwidget)
        self.maxIterations.setGeometry(QtCore.QRect(610, 90, 91, 20))
        self.maxIterations.setObjectName("maxIterations")
        self.biasParam = QtWidgets.QLabel(self.centralwidget)
        self.biasParam.setGeometry(QtCore.QRect(615, 140, 71, 20))
        self.biasParam.setObjectName("biasParam")
        self.submit = QtWidgets.QPushButton(self.centralwidget)
        self.submit.setGeometry(QtCore.QRect(680, 290, 98, 27))
        self.submit.setObjectName("submit")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 823, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.stepSize.setText(_translate("MainWindow", "StepSize"))
        self.maxIterations.setText(_translate("MainWindow", "MaxIterations"))
        self.biasParam.setText(_translate("MainWindow", "BiasParam"))
        self.submit.setText(_translate("MainWindow", "submit"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

