# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sample.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
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
        MainWindow.resize(1126, 653)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.graphicsView = QtGui.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(0, 0, 600, 400))
        self.graphicsView.setStyleSheet(_fromUtf8("background-color: rgb(85, 170, 0);\n"
"background-color: rgb(0, 85, 0);"))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.GoToBall = QtGui.QPushButton(self.centralwidget)
        self.GoToBall.setGeometry(QtCore.QRect(920, 80, 99, 27))
        self.GoToBall.setObjectName(_fromUtf8("GoToBall"))
        self.textBotId = QtGui.QLineEdit(self.centralwidget)
        self.textBotId.setGeometry(QtCore.QRect(790, 80, 113, 27))
        self.textBotId.setObjectName(_fromUtf8("textBotId"))
        self.bot_id = QtGui.QLabel(self.centralwidget)
        self.bot_id.setGeometry(QtCore.QRect(740, 90, 41, 17))
        self.bot_id.setObjectName(_fromUtf8("bot_id"))
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
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.GoToBall.setText(_translate("MainWindow", "GoToBall", None))
        self.bot_id.setText(_translate("MainWindow", "BotId", None))
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

