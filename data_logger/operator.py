# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'operator.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1820, 1080)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(10, 0, 1071, 621))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(10, 850, 251, 101))
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_3 = QtWidgets.QLabel(self.groupBox_2)
        self.label_3.setGeometry(QtCore.QRect(10, 20, 67, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit.setGeometry(QtCore.QRect(80, 20, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton.setGeometry(QtCore.QRect(10, 50, 181, 41))
        self.pushButton.setObjectName("pushButton")
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(1355, 675, 500, 450))
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setGeometry(QtCore.QRect(20, 620, 1061, 151))
        font = QtGui.QFont()
        font.setPointSize(40)
        self.plainTextEdit.setFont(font)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QtCore.QRect(330, 780, 441, 171))
        font = QtGui.QFont()
        font.setPointSize(22)
        self.groupBox_4.setFont(font)
        self.groupBox_4.setObjectName("groupBox_4")
        self.checkBox = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkBox.setGeometry(QtCore.QRect(40, 40, 221, 31))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.checkBox.setFont(font)
        self.checkBox.setObjectName("checkBox")
        self.checkBox_2 = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkBox_2.setGeometry(QtCore.QRect(40, 80, 311, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.checkBox_2.setFont(font)
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_3 = QtWidgets.QCheckBox(self.groupBox_4)
        self.checkBox_3.setGeometry(QtCore.QRect(40, 130, 261, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.checkBox_3.setFont(font)
        self.checkBox_3.setObjectName("checkBox_3")
        self.groupBox_6 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QtCore.QRect(1140, 0, 671, 621))
        self.groupBox_6.setTitle("")
        self.groupBox_6.setObjectName("groupBox_6")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(20, 780, 291, 61))
        font = QtGui.QFont()
        font.setPointSize(21)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setObjectName("lineEdit_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1820, 25))
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
        self.groupBox.setTitle(_translate("MainWindow", "Visualization of environment"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Load participant"))
        self.label_3.setText(_translate("MainWindow", "Number"))
        self.pushButton.setText(_translate("MainWindow", "START EXPERIMENT"))
        self.plainTextEdit.setPlainText(_translate("MainWindow", "START EXPERIMENT"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Success?"))
        self.checkBox.setText(_translate("MainWindow", "Object reached"))
        self.checkBox_2.setText(_translate("MainWindow", "Object not kicked over"))
        self.checkBox_3.setText(_translate("MainWindow", "No collision"))
        self.lineEdit_2.setText(_translate("MainWindow", "0/5 refinements used "))

