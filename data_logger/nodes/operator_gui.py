#!/usr/bin/env python

import rospy, os, sys, csv, rospkg, ast
from data_logger_python.data_logger_python import ParticipantData
from experiment_variables.experiment_variables import ExperimentVariables

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from rviz_python.rviz_python import rvizPython
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, SetObjectMissed, SetObjectMissedResponse,
                                SetObstaclesHit, SetObstaclesHitResponse, ToCsv, ToCsvResponse)

from std_msgs.msg import UInt32, Bool, Byte, String
from data_logger.msg import OperatorGUIinteraction
from experiment.srv import SetText, SetTextResponse

class TextThread(QThread):
    textSignal = pyqtSignal(str)
    def __init__(self, parent=None):
        super(TextThread, self).__init__(parent=parent)
        self.text_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/'
        self.text_file = self.text_path + 'text.txt'

    def run(self):
        while True:
            with open(self.text_file, 'r') as f:
                self.textSignal.emit(f.read())
                self.sleep(1)

class FailureThread(QThread):
    object_missed_signal = pyqtSignal(bool)
    object_kicked_over_signal = pyqtSignal(bool)
    obstacle_hit_signal = pyqtSignal(bool)

    def __init__(self, parent=None):
        super(FailureThread, self).__init__(parent=parent)
        self.text_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/'
        self.object_missed_file = self.text_path + 'object_missed.txt'
        self.object_kicked_over_file = self.text_path + 'object_kicked_over.txt'
        self.obstacle_hit_file = self.text_path + 'obstacle_hit.txt'

    def run(self):
        while True:

            try:
                with open(self.object_missed_file, 'r') as f:
                        self.object_missed_signal.emit(ast.literal_eval(f.read()))
                        self.sleep(1)
                    
                with open(self.object_kicked_over_file, 'r') as f:
                    self.object_kicked_over_signal.emit(ast.literal_eval(f.read()))
                    self.sleep(1)

                with open(self.obstacle_hit_file, 'r') as f:
                    self.obstacle_hit_signal.emit(ast.literal_eval(f.read()))
                    self.sleep(1)
            except SyntaxError as e:
                print("Problem with parsing failure files: " + str(e))
                continue

class NumberOfRefinementsThread(QThread):
    number_of_refinements_signal = pyqtSignal(int)

    def __init__(self, parent=None):
        super(NumberOfRefinementsThread, self).__init__(parent=parent)
        self.text_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/'
        self.number_of_refinements_file = self.text_path + 'number_of_refinements.txt'
    
    def run(self):
        while True:
            
            with open(self.number_of_refinements_file, 'r') as f:
                try:
                    self.number_of_refinements_signal.emit(ast.literal_eval(f.read()))
                except SyntaxError:
                    self.number_of_refinements_signal.emit("0")
                self.sleep(1)
            
class ImageWidget(QWidget):

    def __init__(self, method):
        QWidget.__init__(self)
        self.title = 'PyQt5 image - pythonspot.com'
        self.left = 0
        self.top = 0
        self.width = 640
        self.height = 480
        self.method = method
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        # Create widget
        label = QLabel(self)
        # pixmap = QPixmap('/home/fmeccanici/Documents/thesis/figures/drawio/omni_instruction.png')
        
        if self.method == 'online+pendant':
            pixmap = QPixmap('/home/fmeccanici/Documents/thesis/figures/keyboard_online_instructions2.png')
        elif self.method == 'offline+pendant':
            pixmap = QPixmap('/home/fmeccanici/Documents/thesis/figures/experiment_instructions_keyboard_offline.png')

        pixmap = pixmap.scaled(700, 700, Qt.KeepAspectRatio)

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
        self.text_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/'

        self._operator_gui_interaction_pub = rospy.Publisher('operator_gui_interaction', OperatorGUIinteraction, queue_size=10)
        # self.set_text_service = rospy.Service('operator_gui/set_text', SetText, self._setText)
        # self.set_text_sub = rospy.Subscriber('operator_gui/text', String, self._textCallback)
        
        self.text_thread = TextThread(self)
        self.text_thread.textSignal.connect(self.updateText)
        self.text_thread.start()
        
        self.failure_thread = FailureThread(self)
        self.failure_thread.object_missed_signal.connect(self.updateObjectMissed)
        self.failure_thread.object_kicked_over_signal.connect(self.updateObjectKickedOver)
        self.failure_thread.obstacle_hit_signal.connect(self.updateObstacleHit)
        self.failure_thread.start()

        self.number_of_refinements_thread = NumberOfRefinementsThread(self)
        self.number_of_refinements_thread.number_of_refinements_signal.connect(self.updateNumberOfRefinements)
        self.number_of_refinements_thread.start()

        self.experiment_variables = ExperimentVariables()
        
        self.method = rospy.get_param('~method')
        self.initUI()

        self.show()

    def initUI(self):
        self.setObjectName("MainWindow")
        self.resize(1820, 1080)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(10, 0, 1071, 621))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(10, 850, 251, 101))
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_3 = QLabel(self.groupBox_2)
        self.label_3.setGeometry(QRect(10, 20, 67, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit = QLineEdit(self.groupBox_2)
        self.lineEdit.setGeometry(QRect(80, 20, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton = QPushButton(self.groupBox_2)
        self.pushButton.setGeometry(QRect(10, 50, 181, 41))
        self.pushButton.setObjectName("pushButton")
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(1100, 600, 700, 450))
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.plainTextEdit = QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setGeometry(QRect(20, 620, 1061, 151))
        font = QFont()
        font.setPointSize(40)
        self.plainTextEdit.setFont(font)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(330, 780, 441, 171))
        font = QFont()
        font.setPointSize(22)
        self.groupBox_4.setFont(font)
        self.groupBox_4.setObjectName("groupBox_4")
        self.checkBox = QCheckBox(self.groupBox_4)
        self.checkBox.setGeometry(QRect(40, 40, 221, 31))
        font = QFont()
        font.setPointSize(20)
        self.checkBox.setFont(font)
        self.checkBox.setObjectName("checkBox")
        self.checkBox_2 = QCheckBox(self.groupBox_4)
        self.checkBox_2.setGeometry(QRect(40, 80, 311, 41))
        font = QFont()
        font.setPointSize(20)
        self.checkBox_2.setFont(font)
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_3 = QCheckBox(self.groupBox_4)
        self.checkBox_3.setGeometry(QRect(40, 130, 261, 41))
        font = QFont()
        font.setPointSize(20)
        self.checkBox_3.setFont(font)
        self.checkBox_3.setObjectName("checkBox_3")
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(1075, 0, 671, 621))
        self.groupBox_6.setTitle("")
        self.groupBox_6.setObjectName("groupBox_6")
        self.lineEdit_2 = QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QRect(20, 780, 291, 61))
        font = QFont()
        font.setPointSize(21)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1820, 25))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

        config_file1 = self._rospack.get_path('data_logger') + "/experiment1.rviz"
        config_file2 = self._rospack.get_path('data_logger') + "/experiment2.rviz"

        self.rviz_widget1 = rvizPython(config_file1)
        self.rviz_widget2 = rvizPython(config_file2)

        self.image_widget = ImageWidget(self.method)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout2 = QHBoxLayout()
        self.horizontalLayout3 = QHBoxLayout()

        self.groupBox.setLayout(self.horizontalLayout)
        self.groupBox_6.setLayout(self.horizontalLayout3)

        self.horizontalLayout.addWidget(self.rviz_widget1)
        self.horizontalLayout3.addWidget(self.rviz_widget2)

        self.groupBox_3.setLayout(self.horizontalLayout2)
        
        self.horizontalLayout2.addWidget(self.image_widget)
        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.groupBox.setTitle(_translate("MainWindow", "Visualization of environment"))
        self.groupBox_2.setTitle(_translate("MainWindow", ""))
        self.label_3.setText(_translate("MainWindow", "Number"))
        self.pushButton.setText(_translate("MainWindow", "START EXPERIMENT"))
        self.plainTextEdit.setPlainText(_translate("MainWindow", "START EXPERIMENT"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Success?"))
        self.checkBox.setText(_translate("MainWindow", "Object reached"))
        self.checkBox_2.setText(_translate("MainWindow", "Object not kicked over"))
        self.checkBox_3.setText(_translate("MainWindow", "No collision"))
        self.lineEdit_2.setText(_translate("MainWindow", "0/5 refinements used "))

        self.pushButton.clicked.connect(self.onStartExperimentClick)

    def onStartExperimentClick(self):
        participant_number_msg = Byte(int(self.lineEdit.text()))

        self._operator_gui_interaction_pub.publish(participant_number_msg)

    def updateObjectMissed(self, object_missed):
        if not object_missed:
            self.checkBox.setChecked(1)
        elif object_missed:
            self.checkBox.setChecked(0)

    def updateObjectKickedOver(self, object_kicked_over):
        if not object_kicked_over:
            self.checkBox_2.setChecked(1)
        elif object_kicked_over:
            self.checkBox_2.setChecked(0)

    def updateObstacleHit(self, obstacle_hit):
        if not obstacle_hit:
            self.checkBox_3.setChecked(1)
        elif obstacle_hit:
            self.checkBox_3.setChecked(0)

    def updateNumberOfRefinements(self, number_of_refinements):
        self.lineEdit_2.setText(str(number_of_refinements) + '/' + str(self.experiment_variables.max_refinements) + ' refinements used')

    def updateText(self, text):
        self.plainTextEdit.setPlainText(text)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = OperatorGUI()
    
    sys.exit(app.exec_())
