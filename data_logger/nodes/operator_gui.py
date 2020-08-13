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
                self.number_of_refinements_signal.emit(ast.literal_eval(f.read()))
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

        pixmap = pixmap.scaled(500, 500, Qt.KeepAspectRatio)

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
        self.groupBox.setGeometry(QRect(10, 0, 1071, 701))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(10, 830, 251, 151))
        self.groupBox_2.setObjectName("groupBox_2")
        self.buttonBox = QDialogButtonBox(self.groupBox_2)
        self.buttonBox.setGeometry(QRect(10, 110, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.label = QLabel(self.groupBox_2)
        self.label.setGeometry(QRect(10, 80, 67, 17))
        self.label.setObjectName("label")
        self.label_2 = QLabel(self.groupBox_2)
        self.label_2.setGeometry(QRect(10, 50, 67, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QLabel(self.groupBox_2)
        self.label_3.setGeometry(QRect(10, 20, 67, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit = QLineEdit(self.groupBox_2)
        self.lineEdit.setGeometry(QRect(80, 20, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_3 = QLineEdit(self.groupBox_2)
        self.lineEdit_3.setGeometry(QRect(80, 80, 113, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.radioButton = QRadioButton(self.groupBox_2)
        self.radioButton.setGeometry(QRect(80, 50, 117, 22))
        self.radioButton.setObjectName("radioButton")
        self.radioButton_2 = QRadioButton(self.groupBox_2)
        self.radioButton_2.setGeometry(QRect(160, 50, 117, 22))
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
        self.groupBox_3.setGeometry(QRect(1355, 675, 500, 450))
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")

        if self.method == 'online+pendant':
            self.pushButton = QPushButton(self.centralwidget)
            self.pushButton.setGeometry(QRect(520, 850, 271, 101))
            font = QFont()
            font.setPointSize(40)
            self.pushButton.setFont(font)
            self.pushButton.setObjectName("pushButton")
            self.pushButton_2 = QPushButton(self.centralwidget)
            self.pushButton_2.setGeometry(QRect(810, 850, 271, 101))
            font = QFont()
            font.setPointSize(40)
            self.pushButton_2.setFont(font)
            self.pushButton_2.setObjectName("pushButton_2")

        self.plainTextEdit = QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setGeometry(QRect(20, 730, 1061, 81))
        font = QFont()
        font.setPointSize(40)
        self.plainTextEdit.setFont(font)
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(1090, 730, 291, 281))
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.checkBox = QCheckBox(self.groupBox_4)
        self.checkBox.setGeometry(QRect(20, 10, 201, 31))
        font = QFont()
        font.setPointSize(20)
        self.checkBox.setFont(font)
        self.checkBox.setObjectName("checkBox")
        self.checkBox_2 = QCheckBox(self.groupBox_4)
        self.checkBox_2.setGeometry(QRect(20, 60, 261, 41))
        font = QFont()
        font.setPointSize(20)
        self.checkBox_2.setFont(font)
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_3 = QCheckBox(self.groupBox_4)
        self.checkBox_3.setGeometry(QRect(20, 120, 261, 41))
        font = QFont()
        font.setPointSize(20)
        self.checkBox_3.setFont(font)
        self.checkBox_3.setObjectName("checkBox_3")
        self.lineEdit_2 = QLineEdit(self.groupBox_4)
        self.lineEdit_2.setGeometry(QRect(0, 180, 291, 61))
        font = QFont()
        font.setPointSize(21)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(1075, 0, 671, 701))
        self.groupBox_6.setTitle("")
        self.groupBox_6.setObjectName("groupBox_6")
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
        self.groupBox_2.setTitle(_translate("MainWindow", "Form"))
        self.label.setText(_translate("MainWindow", "Age"))
        self.label_2.setText(_translate("MainWindow", "Sex"))
        self.label_3.setText(_translate("MainWindow", "Number"))
        self.radioButton.setText(_translate("MainWindow", "Ma&le"))
        self.radioButton_2.setText(_translate("MainWindow", "Female"))

        if self.method == 'online+pendant':
            self.pushButton.setText(_translate("MainWindow", "Red"))
            self.pushButton_2.setText(_translate("MainWindow", "Green"))
        
        self.plainTextEdit.setPlainText(_translate("MainWindow", "START EXPERIMENT"))
        self.checkBox.setText(_translate("MainWindow", "Within reach"))
        self.checkBox_2.setText(_translate("MainWindow", "Not kicked over"))
        self.checkBox_3.setText(_translate("MainWindow", "No collision"))
        self.lineEdit_2.setText(_translate("MainWindow", "0/5 refinements used "))

        if self.method == 'online+pendant':
            self.pushButton.setStyleSheet("background-color: red; font: bold 40px; color: black")
            self.pushButton_2.setStyleSheet("background-color: green; font: bold 40px; color: black")

        self.buttonBox.accepted.connect(self.on_ok_click)
        self.radioButton.setChecked(1)

        if self.method == 'online+pendant':
            self.pushButton.clicked.connect(self.onRedClick)
            self.pushButton_2.clicked.connect(self.onGreenClick)
    
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

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = OperatorGUI()
    
    sys.exit(app.exec_())
