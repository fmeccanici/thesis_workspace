#!/usr/bin/env python

import rospy, os, sys, csv, rospkg
from data_logger_python.data_logger_python import ParticipantData

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from rviz_python.rviz_python import rvizPython
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, SetObjectMissed, SetObjectMissedResponse,
                                SetObstaclesHit, SetObstaclesHitResponse, ToCsv, ToCsvResponse)

from std_msgs.msg import UInt32, Bool, Byte
from data_logger.msg import OperatorGUIinteraction

class ImageWidget(QWidget):

    def __init__(self):
        QWidget.__init__(self)
        self.title = 'PyQt5 image - pythonspot.com'
        self.left = 10
        self.top = 10
        self.width = 640
        self.height = 480
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        # Create widget
        label = QLabel(self)
        pixmap = QPixmap('/home/fmeccanici/Documents/thesis/figures/drawio/omni_instruction.png')
        label.setPixmap(pixmap)
        self.resize(pixmap.width(),pixmap.height())
        
        self.show()

class OperatorGUI(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(OperatorGUI, self).__init__(*args, **kwargs)
        # multithreading
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
        ## initialize ROS stuff
        rospy.init_node("operator_gui")
        self._rospack = rospkg.RosPack()
        # self._store_data_service = rospy.Service('store_data', StoreData, self._storeData)

        self._operator_gui_interaction_pub = rospy.Publisher('operator_gui_interaction', OperatorGUIinteraction, queue_size=10)

        self.initUI()
        self.show()

    def initUI(self):
        self.setObjectName("MainWindow")
        self.resize(1920, 1080)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(20, 9, 1271, 671))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(10, 870, 251, 151))
        self.groupBox_2.setObjectName("groupBox_2")
        self.buttonBox = QDialogButtonBox(self.groupBox_2)
        self.buttonBox.setGeometry(QRect(10, 120, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.label = QLabel(self.groupBox_2)
        self.label.setGeometry(QRect(10, 90, 67, 17))
        self.label.setObjectName("label")
        self.label_2 = QLabel(self.groupBox_2)
        self.label_2.setGeometry(QRect(10, 60, 67, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QLabel(self.groupBox_2)
        self.label_3.setGeometry(QRect(10, 30, 67, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit = QLineEdit(self.groupBox_2)
        self.lineEdit.setGeometry(QRect(80, 30, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_3 = QLineEdit(self.groupBox_2)
        self.lineEdit_3.setGeometry(QRect(80, 90, 113, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.radioButton = QRadioButton(self.groupBox_2)
        self.radioButton.setGeometry(QRect(80, 60, 117, 22))
        self.radioButton.setObjectName("radioButton")
        self.radioButton_2 = QRadioButton(self.groupBox_2)
        self.radioButton_2.setGeometry(QRect(160, 60, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.frame = QFrame(self.groupBox_2)
        self.frame.setGeometry(QRect(0, 0, 251, 151))
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setObjectName("frame")
        self.frame.raise_()
        self.buttonBox.raise_()
        self.label.raise_()
        self.label_2.raise_()
        self.label_3.raise_()
        self.lineEdit.raise_()
        self.lineEdit_3.raise_()
        self.radioButton.raise_()
        self.radioButton_2.raise_()
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(1400, 10, 511, 661))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QRect(520, 770, 271, 101))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QRect(810, 770, 271, 101))
        self.pushButton_2.setObjectName("pushButton_2")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1920, 25))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

        config_file = self._rospack.get_path('data_logger') + "/operator_gui.rviz"

        self.rviz_widget = rvizPython(config_file)
        self.image_widget = ImageWidget()
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout2 = QHBoxLayout()

        self.groupBox.setLayout(self.horizontalLayout)
        self.horizontalLayout.addWidget(self.rviz_widget)
        self.groupBox_3.setLayout(self.horizontalLayout2)
        self.horizontalLayout2.addWidget(self.image_widget)
        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Visualization of environment"))
        self.groupBox.setTitle(_translate("MainWindow", "Visualization of environment"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Participant form"))
        self.label.setText(_translate("MainWindow", "Age"))
        self.label_2.setText(_translate("MainWindow", "Sex"))
        self.label_3.setText(_translate("MainWindow", "Number"))
        self.radioButton.setText(_translate("MainWindow", "Ma&le"))
        self.radioButton_2.setText(_translate("MainWindow", "Female"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Instructions"))
        self.pushButton.setText(_translate("MainWindow", "Red"))
        self.pushButton_2.setText(_translate("MainWindow", "Green"))

        self.buttonBox.accepted.connect(self.on_ok_click)
        self.radioButton.setChecked(1)
        self.pushButton.clicked.connect(self.onRedClick)
        self.pushButton_2.clicked.connect(self.onGreenClick)

    def onRedClick(self):
        operator_gui_interaction = OperatorGUIinteraction()
        operator_gui_interaction.refine_prediction = Bool(1)
        operator_gui_interaction.refine_refinement = Bool(0)

        self._operator_gui_interaction_pub.publish(operator_gui_interaction)

    def onGreenClick(self):
        operator_gui_interaction = OperatorGUIinteraction()
        operator_gui_interaction.refine_prediction = Bool(0)
        operator_gui_interaction.refine_refinement = Bool(1)

        self._operator_gui_interaction_pub.publish(operator_gui_interaction)

    def on_ok_click(self):
        operator_gui_interaction = OperatorGUIinteraction()
        operator_gui_interaction.number = Byte(int(self.lineEdit.text()))
        operator_gui_interaction.age = Byte(int(self.lineEdit_3.text()))
        if self.radioButton.isChecked():
            gender = 1
        else:
            gender = 0
        operator_gui_interaction.gender = Bool(gender)
        
        self._operator_gui_interaction_pub.publish(operator_gui_interaction)

        # number = int(self.lineEdit.text())
        # if self.radioButton.isChecked():
        #     sex = 1
        # else:
        #     sex = 0
        # age = int(self.lineEdit_3.text())
        # try: 
        #     rospy.wait_for_service('create_participant')
        #     create_participant = rospy.ServiceProxy('create_participant', CreateParticipant)
        #     number_msg = Byte()
        #     number_msg.data = number
        #     sex_msg = Bool()
        #     sex_msg.data = sex
        #     age_msg = Byte()
        #     age_msg.data = age

        #     resp = create_participant(number_msg, sex_msg, age_msg)     

        # except (rospy.ServiceException, rospy.ROSException) as e:
        #     print("Service call failed: %s" %e)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = OperatorGUI()
    
    sys.exit(app.exec_())
