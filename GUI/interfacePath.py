# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'window.ui'
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
        self.comboBox = QtGui.QComboBox(self.centralwidget)
        self.comboBox.setGeometry(QtCore.QRect(740, 30, 161, 27))
        self.comboBox.setObjectName(_fromUtf8("comboBox"))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.comboBox.addItem(_fromUtf8(""))
        self.goToBallFsm = QtGui.QPushButton(self.centralwidget)
        self.goToBallFsm.setGeometry(QtCore.QRect(920, 110, 121, 27))
        self.goToBallFsm.setObjectName(_fromUtf8("goToBallFsm"))
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
        self.actionRRTStar = QtGui.QAction(MainWindow)
        self.actionRRTStar.setObjectName(_fromUtf8("actionRRTStar"))
        self.actionRRT = QtGui.QAction(MainWindow)
        self.actionRRT.setObjectName(_fromUtf8("actionRRT"))
        self.menubar.addAction(self.menuDIFFERENT_RRT.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.GoToBall.setText(_translate("MainWindow", "GoToBall", None))
        self.bot_id.setText(_translate("MainWindow", "BotId", None))
        self.comboBox.setItemText(0, _translate("MainWindow", "PRM", None))
        self.comboBox.setItemText(1, _translate("MainWindow", "RRT", None))
        self.comboBox.setItemText(2, _translate("MainWindow", "RRTConnect", None))
        self.comboBox.setItemText(3, _translate("MainWindow", "RRTStar", None))
        self.comboBox.setItemText(4, _translate("MainWindow", "LBTRRT", None))
        self.comboBox.setItemText(5, _translate("MainWindow", "LazyRRT", None))
        self.comboBox.setItemText(6, _translate("MainWindow", "TRRT", None))
        self.comboBox.setItemText(7, _translate("MainWindow", "pRRT", None))
        self.comboBox.setItemText(8, _translate("MainWindow", "EST", None))
        self.goToBallFsm.setText(_translate("MainWindow", "GoToBall(FSM)", None))
        self.menuDIFFERENT_RRT.setTitle(_translate("MainWindow", "DIFFERENT_RRT", None))
        self.actionRRT_CONNECT.setText(_translate("MainWindow", "RRT_CONNECT", None))
        self.actionRRTStar.setText(_translate("MainWindow", "RRT_STAR", None))
        self.actionRRT.setText(_translate("MainWindow", "RRT", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

