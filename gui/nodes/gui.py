#!/usr/bin/env python

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# other packages
import rospy, sys, roslaunch, rospkg, os, random, time

from rviz_python.rviz_python import rvizPython

# ROS messages
from learning_from_demonstration.srv import (AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, 
                                            SetObject, SetObjectResponse, GetContext, GetContextResponse, 
                                            GoToPose, GoToPoseResponse, ExecuteTrajectory, ExecuteTrajectoryResponse,
                                            GetObjectPosition, GetObjectPositionResponse, WelfordUpdate, 
                                            WelfordUpdateResponse, SetTeachingMode, SetTeachingModeResponse, 
                                            BuildInitialModel, BuildInitialModelResponse, 
                                            GetEEPose, GetEEPoseResponse, SetPath)
                                            
from trajectory_refinement.srv import RefineTrajectory, RefineTrajectoryResponse, CalibrateMasterPose
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point
from std_msgs.msg import String, Bool, Byte, UInt32
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from promp_context_ros.msg import prompTraj
from trajectory_visualizer.srv import VisualizeTrajectory, VisualizeTrajectoryResponse, ClearTrajectories, ClearTrajectoriesResponse
from trajectory_visualizer.msg import TrajectoryVisualization

from learning_from_demonstration_python.trajectory_parser import trajectoryParser
from data_logger.srv import (CreateParticipant, CreateParticipantResponse, AddRefinement, AddRefinementResponse,
                                SetPrediction, SetPredictionResponse, IncrementObjectMissed, IncrementObjectMissedResponse,
                                IncrementObstaclesHit, IncrementObstaclesHitResponse, IncrementNumberOfUpdates, IncrementNumberOfUpdatesResponse,
                                SetNumberOfUpdates, SetNumberOfUpdatesResponse, ToCsv, ToCsvResponse)

# class that enables multithreading with Qt
class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and 
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    # use custom function 
    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''
        self.fn(*self.args, **self.kwargs)

class experimentGUI(QMainWindow):

    def __init__(self, *args, **kwargs):

        # inheretance on QMainWindow class
        super(experimentGUI, self).__init__(*args, **kwargs)

        # multithreading
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        ## initialize ROS stuff
        rospy.init_node("experiment_gui")

        self._rospack = rospkg.RosPack()
        self.parser = trajectoryParser()

        self.launch_files = {'learning_from_demonstration':'learning_from_demonstration.launch', 
                            'trajectory_refinement':'trajectory_refinement.launch',
                            'learning_from_demonstration':'trajectory_teaching.launch'}
        self.nodes = {}

        # initialize Qt GUI
        self.initGUI()
        self.show()

    def initGUI(self):
        self.setObjectName("Online teaching GUI")
        self.resize(1920, 1080)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(310, 690, 351, 341))
        self.groupBox_2.setObjectName("groupBox_2")
        self.pushButton_6 = QPushButton(self.groupBox_2)
        self.pushButton_6.setGeometry(QRect(0, 120, 150, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_21 = QPushButton(self.groupBox_2)
        self.pushButton_21.setGeometry(QRect(0, 30, 150, 27))
        self.pushButton_21.setObjectName("pushButton_21")
        self.pushButton_22 = QPushButton(self.groupBox_2)
        self.pushButton_22.setGeometry(QRect(0, 90, 150, 27))
        self.pushButton_22.setObjectName("pushButton_22")
        self.pushButton_23 = QPushButton(self.groupBox_2)
        self.pushButton_23.setGeometry(QRect(0, 60, 150, 27))
        self.pushButton_23.setObjectName("pushButton_23")
        self.pushButton_10 = QPushButton(self.groupBox_2)
        self.pushButton_10.setGeometry(QRect(180, 30, 150, 27))
        self.pushButton_10.setObjectName("pushButton_10")
        self.radioButton_2 = QRadioButton(self.groupBox_2)
        self.radioButton_2.setGeometry(QRect(40, 150, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.radioButton_3 = QRadioButton(self.groupBox_2)
        self.radioButton_3.setGeometry(QRect(40, 180, 117, 22))
        self.radioButton_3.setObjectName("radioButton_3")
        self.pushButton_24 = QPushButton(self.groupBox_2)
        self.pushButton_24.setGeometry(QRect(180, 60, 150, 27))
        self.pushButton_24.setObjectName("pushButton_24")
        self.groupBox_7 = QGroupBox(self.groupBox_2)
        self.groupBox_7.setGeometry(QRect(180, 100, 120, 131))
        self.groupBox_7.setObjectName("groupBox_7")
        self.radioButton_8 = QRadioButton(self.groupBox_7)
        self.radioButton_8.setGeometry(QRect(10, 30, 117, 22))
        self.radioButton_8.setObjectName("radioButton_8")
        self.radioButton_9 = QRadioButton(self.groupBox_7)
        self.radioButton_9.setGeometry(QRect(10, 60, 117, 22))
        self.radioButton_9.setObjectName("radioButton_9")
        self.lineEdit_17 = QLineEdit(self.groupBox_7)
        self.lineEdit_17.setGeometry(QRect(40, 90, 41, 27))
        self.lineEdit_17.setObjectName("lineEdit_17")
        self.pushButton_26 = QPushButton(self.groupBox_2)
        self.pushButton_26.setGeometry(QRect(0, 280, 150, 27))
        self.pushButton_26.setObjectName("pushButton_26")
        self.lineEdit_18 = QLineEdit(self.groupBox_2)
        self.lineEdit_18.setGeometry(QRect(0, 310, 171, 27))
        self.lineEdit_18.setObjectName("lineEdit_18")
        self.frame_2 = QFrame(self.groupBox_2)
        self.frame_2.setGeometry(QRect(0, 0, 331, 341))
        self.frame_2.setFrameShape(QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.lineEdit_19 = QLineEdit(self.frame_2)
        self.lineEdit_19.setGeometry(QRect(40, 210, 41, 27))
        self.lineEdit_19.setObjectName("lineEdit_19")
        self.frame_2.raise_()
        self.pushButton_6.raise_()
        self.pushButton_21.raise_()
        self.pushButton_22.raise_()
        self.pushButton_23.raise_()
        self.pushButton_10.raise_()
        self.radioButton_2.raise_()
        self.radioButton_3.raise_()
        self.pushButton_24.raise_()
        self.groupBox_7.raise_()
        self.pushButton_26.raise_()
        self.lineEdit_18.raise_()
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(570, 690, 511, 291))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton_7 = QPushButton(self.groupBox_3)
        self.pushButton_7.setGeometry(QRect(150, 20, 99, 27))
        self.pushButton_7.setObjectName("pushButton_7")
        self.label_5 = QLabel(self.groupBox_3)
        self.label_5.setGeometry(QRect(114, 59, 16, 17))
        self.label_5.setObjectName("label_5")
        self.lineEdit_5 = QLineEdit(self.groupBox_3)
        self.lineEdit_5.setGeometry(QRect(135, 59, 146, 27))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.label_4 = QLabel(self.groupBox_3)
        self.label_4.setGeometry(QRect(114, 92, 16, 17))
        self.label_4.setObjectName("label_4")
        self.lineEdit_6 = QLineEdit(self.groupBox_3)
        self.lineEdit_6.setGeometry(QRect(135, 92, 146, 27))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.label_6 = QLabel(self.groupBox_3)
        self.label_6.setGeometry(QRect(114, 125, 16, 17))
        self.label_6.setObjectName("label_6")
        self.lineEdit_4 = QLineEdit(self.groupBox_3)
        self.lineEdit_4.setGeometry(QRect(135, 125, 146, 27))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.pushButton_5 = QPushButton(self.groupBox_3)
        self.pushButton_5.setGeometry(QRect(340, 26, 85, 27))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_8 = QPushButton(self.groupBox_3)
        self.pushButton_8.setGeometry(QRect(340, 59, 152, 27))
        self.pushButton_8.setObjectName("pushButton_8")
        self.pushButton_9 = QPushButton(self.groupBox_3)
        self.pushButton_9.setGeometry(QRect(340, 92, 151, 27))
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_13 = QPushButton(self.groupBox_3)
        self.pushButton_13.setGeometry(QRect(340, 130, 151, 27))
        self.pushButton_13.setObjectName("pushButton_13")
        self.label_20 = QLabel(self.groupBox_3)
        self.label_20.setGeometry(QRect(140, 170, 111, 16))
        self.label_20.setObjectName("label_20")
        self.pushButton_12 = QPushButton(self.groupBox_3)
        self.pushButton_12.setGeometry(QRect(150, 190, 90, 27))
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_11 = QPushButton(self.groupBox_3)
        self.pushButton_11.setGeometry(QRect(150, 220, 88, 27))
        self.pushButton_11.setObjectName("pushButton_11")
        self.pushButton_14 = QPushButton(self.groupBox_3)
        self.pushButton_14.setGeometry(QRect(150, 250, 90, 27))
        self.pushButton_14.setObjectName("pushButton_14")
        self.frame_3 = QFrame(self.groupBox_3)
        self.frame_3.setGeometry(QRect(100, 0, 401, 281))
        self.frame_3.setFrameShape(QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.frame_3.raise_()
        self.pushButton_5.raise_()
        self.pushButton_8.raise_()
        self.pushButton_9.raise_()
        self.label_5.raise_()
        self.lineEdit_5.raise_()
        self.label_4.raise_()
        self.lineEdit_6.raise_()
        self.label_6.raise_()
        self.lineEdit_4.raise_()
        self.pushButton_7.raise_()
        self.pushButton_13.raise_()
        self.label_20.raise_()
        self.pushButton_12.raise_()
        self.pushButton_11.raise_()
        self.pushButton_14.raise_()
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(20, 690, 271, 181))
        self.groupBox.setObjectName("groupBox")
        self.label = QLabel(self.groupBox)
        self.label.setGeometry(QRect(0, 50, 16, 17))
        self.label.setObjectName("label")
        self.lineEdit = QLineEdit(self.groupBox)
        self.lineEdit.setGeometry(QRect(21, 51, 39, 26))
        self.lineEdit.setObjectName("lineEdit")
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setGeometry(QRect(0, 83, 16, 17))
        self.label_2.setObjectName("label_2")
        self.buttonBox = QDialogButtonBox(self.groupBox)
        self.buttonBox.setGeometry(QRect(0, 145, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.lineEdit_2 = QLineEdit(self.groupBox)
        self.lineEdit_2.setGeometry(QRect(21, 83, 39, 26))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_3 = QLineEdit(self.groupBox)
        self.lineEdit_3.setGeometry(QRect(21, 115, 39, 25))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.label_3 = QLabel(self.groupBox)
        self.label_3.setGeometry(QRect(0, 116, 16, 17))
        self.label_3.setObjectName("label_3")
        self.pushButton_15 = QPushButton(self.groupBox)
        self.pushButton_15.setGeometry(QRect(182, 145, 85, 27))
        self.pushButton_15.setObjectName("pushButton_15")
        self.label_16 = QLabel(self.groupBox)
        self.label_16.setGeometry(QRect(0, 30, 67, 17))
        self.label_16.setObjectName("label_16")
        self.label_21 = QLabel(self.groupBox)
        self.label_21.setGeometry(QRect(90, 30, 67, 17))
        self.label_21.setObjectName("label_21")
        self.lineEdit_14 = QLineEdit(self.groupBox)
        self.lineEdit_14.setGeometry(QRect(101, 51, 39, 26))
        self.lineEdit_14.setObjectName("lineEdit_14")
        self.lineEdit_15 = QLineEdit(self.groupBox)
        self.lineEdit_15.setGeometry(QRect(101, 83, 39, 26))
        self.lineEdit_15.setObjectName("lineEdit_15")
        self.lineEdit_16 = QLineEdit(self.groupBox)
        self.lineEdit_16.setGeometry(QRect(101, 115, 39, 25))
        self.lineEdit_16.setObjectName("lineEdit_16")
        self.frame = QFrame(self.groupBox)
        self.frame.setGeometry(QRect(0, 0, 271, 181))
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setObjectName("frame")
        self.frame.raise_()
        self.label.raise_()
        self.lineEdit.raise_()
        self.label_2.raise_()
        self.buttonBox.raise_()
        self.lineEdit_2.raise_()
        self.lineEdit_3.raise_()
        self.label_3.raise_()
        self.pushButton_15.raise_()
        self.label_16.raise_()
        self.label_21.raise_()
        self.lineEdit_14.raise_()
        self.lineEdit_15.raise_()
        self.lineEdit_16.raise_()
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(1120, 690, 431, 301))
        self.groupBox_6.setObjectName("groupBox_6")
        self.label_7 = QLabel(self.groupBox_6)
        self.label_7.setGeometry(QRect(11, 102, 16, 17))
        self.label_7.setObjectName("label_7")
        self.label_8 = QLabel(self.groupBox_6)
        self.label_8.setGeometry(QRect(10, 68, 16, 17))
        self.label_8.setObjectName("label_8")
        self.label_9 = QLabel(self.groupBox_6)
        self.label_9.setGeometry(QRect(10, 35, 16, 17))
        self.label_9.setObjectName("label_9")
        self.label_10 = QLabel(self.groupBox_6)
        self.label_10.setGeometry(QRect(1, 201, 24, 17))
        self.label_10.setObjectName("label_10")
        self.label_11 = QLabel(self.groupBox_6)
        self.label_11.setGeometry(QRect(1, 168, 24, 17))
        self.label_11.setObjectName("label_11")
        self.label_12 = QLabel(self.groupBox_6)
        self.label_12.setGeometry(QRect(1, 135, 24, 17))
        self.label_12.setObjectName("label_12")
        self.label_13 = QLabel(self.groupBox_6)
        self.label_13.setGeometry(QRect(0, 230, 28, 21))
        self.label_13.setObjectName("label_13")
        self.layoutWidget = QWidget(self.groupBox_6)
        self.layoutWidget.setGeometry(QRect(31, 35, 148, 227))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout = QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.lineEdit_9 = QLineEdit(self.layoutWidget)
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.verticalLayout.addWidget(self.lineEdit_9)
        self.lineEdit_7 = QLineEdit(self.layoutWidget)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.verticalLayout.addWidget(self.lineEdit_7)
        self.lineEdit_8 = QLineEdit(self.layoutWidget)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.verticalLayout.addWidget(self.lineEdit_8)
        self.lineEdit_12 = QLineEdit(self.layoutWidget)
        self.lineEdit_12.setObjectName("lineEdit_12")
        self.verticalLayout.addWidget(self.lineEdit_12)
        self.lineEdit_11 = QLineEdit(self.layoutWidget)
        self.lineEdit_11.setObjectName("lineEdit_11")
        self.verticalLayout.addWidget(self.lineEdit_11)
        self.lineEdit_10 = QLineEdit(self.layoutWidget)
        self.lineEdit_10.setObjectName("lineEdit_10")
        self.verticalLayout.addWidget(self.lineEdit_10)
        self.lineEdit_13 = QLineEdit(self.layoutWidget)
        self.lineEdit_13.setObjectName("lineEdit_13")
        self.verticalLayout.addWidget(self.lineEdit_13)
        self.pushButton_16 = QPushButton(self.groupBox_6)
        self.pushButton_16.setGeometry(QRect(10, 270, 85, 27))
        self.pushButton_16.setObjectName("pushButton_16")
        self.pushButton_17 = QPushButton(self.groupBox_6)
        self.pushButton_17.setGeometry(QRect(100, 270, 85, 27))
        self.pushButton_17.setObjectName("pushButton_17")
        self.label_14 = QLabel(self.groupBox_6)
        self.label_14.setGeometry(QRect(190, 160, 67, 17))
        self.label_14.setObjectName("label_14")
        self.label_15 = QLabel(self.groupBox_6)
        self.label_15.setGeometry(QRect(190, 20, 101, 17))
        self.label_15.setObjectName("label_15")
        self.pushButton_18 = QPushButton(self.groupBox_6)
        self.pushButton_18.setGeometry(QRect(190, 40, 85, 27))
        self.pushButton_18.setObjectName("pushButton_18")
        self.pushButton_19 = QPushButton(self.groupBox_6)
        self.pushButton_19.setGeometry(QRect(190, 70, 85, 27))
        self.pushButton_19.setObjectName("pushButton_19")
        self.label_19 = QLabel(self.groupBox_6)
        self.label_19.setGeometry(QRect(310, 20, 101, 17))
        self.label_19.setObjectName("label_19")
        self.pushButton_20 = QPushButton(self.groupBox_6)
        self.pushButton_20.setGeometry(QRect(300, 40, 85, 27))
        self.pushButton_20.setObjectName("pushButton_20")
        self.radioButton = QRadioButton(self.groupBox_6)
        self.radioButton.setGeometry(QRect(200, 180, 41, 22))
        self.radioButton.setObjectName("radioButton")
        self.radioButton_4 = QRadioButton(self.groupBox_6)
        self.radioButton_4.setGeometry(QRect(200, 210, 117, 22))
        self.radioButton_4.setObjectName("radioButton_4")
        self.radioButton_5 = QRadioButton(self.groupBox_6)
        self.radioButton_5.setGeometry(QRect(200, 240, 117, 22))
        self.radioButton_5.setObjectName("radioButton_5")
        self.pushButton_25 = QPushButton(self.groupBox_6)
        self.pushButton_25.setGeometry(QRect(190, 100, 85, 27))
        self.pushButton_25.setObjectName("pushButton_25")
        self.radioButton_6 = QRadioButton(self.groupBox_6)
        self.radioButton_6.setGeometry(QRect(271, 181, 158, 22))
        self.radioButton_6.setObjectName("radioButton_6")
        self.radioButton_7 = QRadioButton(self.groupBox_6)
        self.radioButton_7.setGeometry(QRect(271, 209, 171, 22))
        self.radioButton_7.setObjectName("radioButton_7")
        self.radioButton_10 = QRadioButton(self.groupBox_6)
        self.radioButton_10.setGeometry(QRect(200, 270, 117, 22))
        self.radioButton_10.setObjectName("radioButton_10")
        self.frame_4 = QFrame(self.groupBox_6)
        self.frame_4.setGeometry(QRect(0, 0, 431, 301))
        self.frame_4.setFrameShape(QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.frame_4.raise_()
        self.label_7.raise_()
        self.label_8.raise_()
        self.label_9.raise_()
        self.label_10.raise_()
        self.label_11.raise_()
        self.label_12.raise_()
        self.label_13.raise_()
        self.layoutWidget.raise_()
        self.pushButton_16.raise_()
        self.pushButton_17.raise_()
        self.label_14.raise_()
        self.label_15.raise_()
        self.pushButton_18.raise_()
        self.pushButton_19.raise_()
        self.label_19.raise_()
        self.pushButton_20.raise_()
        self.radioButton.raise_()
        self.radioButton_4.raise_()
        self.radioButton_5.raise_()
        self.pushButton_25.raise_()
        self.radioButton_6.raise_()
        self.radioButton_7.raise_()
        self.radioButton_10.raise_()
        self.groupBox_5 = QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QRect(20, 880, 271, 121))
        self.groupBox_5.setObjectName("groupBox_5")
        self.label_17 = QLabel(self.groupBox_5)
        self.label_17.setGeometry(QRect(40, 37, 31, 17))
        self.label_17.setObjectName("label_17")
        self.pushButton_3 = QPushButton(self.groupBox_5)
        self.pushButton_3.setGeometry(QRect(10, 57, 90, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QPushButton(self.groupBox_5)
        self.pushButton_4.setGeometry(QRect(10, 87, 88, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.label_18 = QLabel(self.groupBox_5)
        self.label_18.setGeometry(QRect(170, 37, 91, 17))
        self.label_18.setObjectName("label_18")
        self.pushButton = QPushButton(self.groupBox_5)
        self.pushButton.setGeometry(QRect(170, 57, 90, 27))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QPushButton(self.groupBox_5)
        self.pushButton_2.setGeometry(QRect(170, 90, 88, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.groupBox_8 = QGroupBox(self.centralwidget)
        self.groupBox_8.setGeometry(QRect(1580, 690, 331, 331))
        self.groupBox_8.setObjectName("groupBox_8")
        self.groupBox_9 = QGroupBox(self.groupBox_8)
        self.groupBox_9.setGeometry(QRect(0, 30, 91, 261))
        self.groupBox_9.setObjectName("groupBox_9")
        self.radioButton_11 = QRadioButton(self.groupBox_9)
        self.radioButton_11.setGeometry(QRect(10, 30, 117, 22))
        self.radioButton_11.setObjectName("radioButton_11")
        self.radioButton_12 = QRadioButton(self.groupBox_9)
        self.radioButton_12.setGeometry(QRect(10, 60, 117, 22))
        self.radioButton_12.setObjectName("radioButton_12")
        self.radioButton_13 = QRadioButton(self.groupBox_9)
        self.radioButton_13.setGeometry(QRect(10, 90, 117, 22))
        self.radioButton_13.setObjectName("radioButton_13")
        self.pushButton_27 = QPushButton(self.groupBox_9)
        self.pushButton_27.setGeometry(QRect(0, 200, 91, 27))
        self.pushButton_27.setObjectName("pushButton_27")
        self.pushButton_28 = QPushButton(self.groupBox_9)
        self.pushButton_28.setGeometry(QRect(0, 230, 91, 27))
        self.pushButton_28.setObjectName("pushButton_28")
        self.pushButton_29 = QPushButton(self.groupBox_9)
        self.pushButton_29.setGeometry(QRect(0, 130, 91, 27))
        self.pushButton_29.setObjectName("pushButton_29")
        self.pushButton_30 = QPushButton(self.groupBox_9)
        self.pushButton_30.setGeometry(QRect(0, 160, 91, 27))
        self.pushButton_30.setObjectName("pushButton_30")
        self.groupBox_10 = QGroupBox(self.groupBox_8)
        self.groupBox_10.setGeometry(QRect(90, 30, 111, 121))
        self.groupBox_10.setObjectName("groupBox_10")
        self.radioButton_14 = QRadioButton(self.groupBox_10)
        self.radioButton_14.setGeometry(QRect(10, 30, 41, 22))
        self.radioButton_14.setObjectName("radioButton_14")
        self.radioButton_15 = QRadioButton(self.groupBox_10)
        self.radioButton_15.setGeometry(QRect(10, 60, 117, 22))
        self.radioButton_15.setObjectName("radioButton_15")
        self.radioButton_16 = QRadioButton(self.groupBox_10)
        self.radioButton_16.setGeometry(QRect(10, 90, 117, 22))
        self.radioButton_16.setObjectName("radioButton_16")
        self.radioButton_17 = QRadioButton(self.groupBox_10)
        self.radioButton_17.setGeometry(QRect(60, 30, 117, 22))
        self.radioButton_17.setObjectName("radioButton_17")
        self.radioButton_18 = QRadioButton(self.groupBox_10)
        self.radioButton_18.setGeometry(QRect(60, 60, 117, 22))
        self.radioButton_18.setObjectName("radioButton_18")
        self.radioButton_19 = QRadioButton(self.groupBox_10)
        self.radioButton_19.setGeometry(QRect(60, 90, 117, 22))
        self.radioButton_19.setObjectName("radioButton_19")
        self.frame_5 = QFrame(self.groupBox_8)
        self.frame_5.setGeometry(QRect(0, 0, 331, 311))
        self.frame_5.setFrameShape(QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.pushButton_31 = QPushButton(self.frame_5)
        self.pushButton_31.setGeometry(QRect(100, 160, 91, 27))
        self.pushButton_31.setObjectName("pushButton_31")
        self.lineEdit_20 = QLineEdit(self.frame_5)
        self.lineEdit_20.setGeometry(QRect(180, 190, 31, 27))
        self.lineEdit_20.setObjectName("lineEdit_20")
        self.label_22 = QLabel(self.frame_5)
        self.label_22.setGeometry(QRect(100, 190, 91, 20))
        self.label_22.setObjectName("label_22")
        self.radioButton_20 = QRadioButton(self.frame_5)
        self.radioButton_20.setGeometry(QRect(100, 220, 91, 22))
        self.radioButton_20.setObjectName("radioButton_20")
        self.radioButton_21 = QRadioButton(self.frame_5)
        self.radioButton_21.setGeometry(QRect(100, 240, 101, 22))
        self.radioButton_21.setObjectName("radioButton_21")
        self.pushButton_32 = QPushButton(self.frame_5)
        self.pushButton_32.setGeometry(QRect(220, 160, 91, 27))
        self.pushButton_32.setObjectName("pushButton_32")
        self.pushButton_33 = QPushButton(self.frame_5)
        self.pushButton_33.setGeometry(QRect(220, 190, 101, 27))
        self.pushButton_33.setObjectName("pushButton_33")
        self.pushButton_34 = QPushButton(self.frame_5)
        self.pushButton_34.setGeometry(QRect(220, 220, 101, 27))
        self.pushButton_34.setObjectName("pushButton_34")
        self.frame_6 = QFrame(self.frame_5)
        self.frame_6.setGeometry(QRect(100, 260, 120, 41))
        self.frame_6.setFrameShape(QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.radioButton_22 = QRadioButton(self.frame_6)
        self.radioButton_22.setGeometry(QRect(20, 0, 117, 22))
        self.radioButton_22.setObjectName("radioButton_22")
        self.radioButton_23 = QRadioButton(self.frame_6)
        self.radioButton_23.setGeometry(QRect(20, 20, 117, 22))
        self.radioButton_23.setObjectName("radioButton_23")
        self.frame_5.raise_()
        self.groupBox_9.raise_()
        self.groupBox_10.raise_()
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(10, 10, 1821, 661))
        self.groupBox_4.setObjectName("groupBox_4")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1920, 25))
        self.menubar.setObjectName("menubar")
        self.menuOnline_teaching_GUI = QMenu(self.menubar)
        self.menuOnline_teaching_GUI.setObjectName("menuOnline_teaching_GUI")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuOnline_teaching_GUI.menuAction())

        # initialize default radio buttons
        self.radioButton_8.setChecked(True)
        self.radioButton_3.setChecked(True)
        self.radioButton_11.setChecked(True)
        self.radioButton_14.setChecked(True)

        # object position
        self.lineEdit.setText("0.8")
        self.lineEdit_2.setText("0.2")
        self.lineEdit_3.setText("0.9")

        # obstacle position
        self.lineEdit_14.setText("0.7")
        self.lineEdit_15.setText("0")
        self.lineEdit_16.setText("0.7")

        self.lineEdit_17.setText("10")
        config_file = self._rospack.get_path('gui') + "/gui.rviz"

        self.rviz_widget = rvizPython(config_file)
        self.horizontalLayout_6 = QHBoxLayout()
        self.groupBox_4.setLayout(self.horizontalLayout_6)
        self.horizontalLayout_6.addWidget(self.rviz_widget)
        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)


    def on_load_trajectory_click(self):
        traj_file = str(self.lineEdit_18.text())
        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/' 
        try:
            ref_traj = self.parser.openTrajectoryFile(traj_file, path)
            self.refined_trajectory = self.parser.predicted_trajectory_to_prompTraj_message(ref_traj, self.parser.point_to_list(self.context))
        except Exception as e:
            rospy.loginfo(e)

    def start_node(self, package, launch_file):
        
        if launch_file not in self.nodes: 
            abs_path = self._rospack.get_path(package) + "/launch/" + launch_file
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])

            # needed to be able to stop the node
            self.nodes[launch_file] = launch

            # start node
            self.nodes[launch_file].start()
            
        else:
            # we need to delete the previous node from dict
            # else it gives an error
            del self.nodes[launch_file]

            # and append a new one again
            abs_path = self._rospack.get_path(package) + "/launch/" + launch_file
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])

            # needed to be able to stop the node
            self.nodes[launch_file] = launch

            self.nodes[launch_file].start()
            rospy.loginfo( ("Started {} ").format(launch_file) )


    def stop_node(self, launch_file):
        # look through dictionary to find corresponding launch object
        try:
            self.nodes[launch_file].shutdown()
        except AttributeError, KeyError:
            rospy.loginfo( ("Node not launched yet") )
    
    def on_refine_refinement_click(self):
        try:
            rospy.wait_for_service('refine_trajectory', timeout=2.0)

            refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)

            if self.radioButton_9.isChecked():
                self.T_desired = float(self.lineEdit_17.text())
            elif self.radioButton_8.isChecked():
                self.T_desired = 0.0

            resp = refine_trajectory(self.refined_trajectory, self.T_desired)
            self.refined_trajectory = resp.refined_trajectory
            rospy.loginfo("Got a refined trajectory")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def on_refine_prediction_click(self):
        try:
            # rospy.wait_for_service('refine_trajectory', timeout=2.0)

            refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
            
            if self.radioButton_9.isChecked():
                self.T_desired = float(self.lineEdit_17.text())
            elif self.radioButton_8.isChecked():
                self.T_desired = 0.0
            try:
                resp = refine_trajectory(self.prediction, self.T_desired)
                self.refined_trajectory = resp.refined_trajectory
                # f = open("/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/refined_trajectory.txt", 'w+')
                # f.write(str(self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)[0]))
                # f.close()


                # print(self.refined_trajectory.times)
                rospy.loginfo("Got a refined trajectory")

            except AttributeError as e:
                rospy.loginfo(("No refined trajectory available yet!: {}").format(e))

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)
    
    def on_set_obstacle_position_click(self):
        try:
            obstacle_position = ModelState()
            obstacle_position.model_name = 'coke_can'
            obstacle_position.pose.position.x = float(self.lineEdit_14.text())
            obstacle_position.pose.position.y = float(self.lineEdit_15.text())
            obstacle_position.pose.position.z = float(self.lineEdit_16.text())
            obstacle_position.pose.orientation.x = 0
            obstacle_position.pose.orientation.y = 0
            obstacle_position.pose.orientation.z = 0
            obstacle_position.pose.orientation.w = 1
        except ValueError:
            rospy.loginfo("Invalid value for position!")
        try:
            rospy.wait_for_service('/gazebo/set_model_state')

            set_obstacle = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_obstacle(obstacle_position)
            return resp.success

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)


    def on_cancel_position_click(self):
        # object position
        self.lineEdit.setText("0.8")
        self.lineEdit_2.setText("0.3")
        self.lineEdit_3.setText("0.9")

        # obstacle position
        self.lineEdit_14.setText("0.8")
        self.lineEdit_15.setText("0")
        self.lineEdit_16.setText("0.7")

    def on_set_object_position_click(self):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'
            object_position.pose.position.x = float(self.lineEdit.text())
            object_position.pose.position.y = float(self.lineEdit_2.text())
            object_position.pose.position.z = float(self.lineEdit_3.text())
            object_position.pose.orientation.x = 0
            object_position.pose.orientation.y = 0
            object_position.pose.orientation.z = 0
            object_position.pose.orientation.w = 1
        except ValueError:
            rospy.loginfo("Invalid value for position!")

        try:
            rospy.wait_for_service('/gazebo/set_model_state')

            set_object = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_object(object_position)
            return resp.success

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def on_clear_trajectories_click(self):
        try:
            rospy.wait_for_service('clear_trajectories', timeout=2.0)

            clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)

            resp = clear_trajectories()
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_visualize_refinement_click(self):
        # visualize refined trajectory
        try:
            rospy.wait_for_service('visualize_trajectory', timeout=2.0)

            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()
            try:
                visualization_msg.pose_array = self.refined_trajectory.poses
            
            except AttributeError:
                rospy.loginfo("No prediction made yet!")
            
            visualization_msg.r = 0.0
            visualization_msg.g = 1.0
            visualization_msg.b = 0.0

            resp = visualize_trajectory(visualization_msg)
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    

    def on_visualize_prediction_click(self):

        # visualize trajectory
        try:
            rospy.wait_for_service('visualize_trajectory', timeout=2.0)

            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()
            try:
                visualization_msg.pose_array = self.prediction.poses
            
            except AttributeError:
                rospy.loginfo("No prediction made yet!")
            
            visualization_msg.r = 1.0
            visualization_msg.g = 0.0
            visualization_msg.b = 0.0

            resp = visualize_trajectory(visualization_msg)
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_get_context_click(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context
            self.lineEdit_5.setText(str(round(self.context.x, 2)))
            self.lineEdit_6.setText(str(round(self.context.y, 2)))
            self.lineEdit_4.setText(str(round(self.context.z, 2)))

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)       

    def on_predict_click(self):
        # make prediction using this context

        try:
            rospy.wait_for_service('make_prediction', timeout=2.0)

            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            try:
                resp = make_prediction(self.context)
                self.prediction = resp.prediction

            except AttributeError:
                rospy.loginfo("Context not yet extracted!")
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def on_initialize_head_joints_click(self):
        package = 'teleop_control'
        launch_file = 'set_head_joints.launch'

        abs_path = self._rospack.get_path(package) + "/launch/" + launch_file
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)


        self.lfd_launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])
        self.lfd_launch.start()

        rospy.loginfo( ("Started {} ").format(launch_file) )

    # multithread for executing trajectories
    # needed since otherwise the GUI will freeze
    def use_multithread(self, function):
        worker = Worker(function)
        self.threadpool.start(worker)
    
    def on_execute_refinement_click(self):
        try:
            rospy.wait_for_service('execute_trajectory', timeout=2.0)

            execute_refinement = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)

            if self.radioButton_9.isChecked():
                self.T_desired = float(self.lineEdit_17.text())
            else:
                self.T_desired = None

            try:
                resp = execute_refinement(self.refined_trajectory, self.T_desired)

            except AttributeError:
                rospy.loginfo("No refined trajectory available yet!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)   
            

    def on_execute_prediction_click(self):
        
        try:
            rospy.wait_for_service('execute_trajectory', timeout=2.0)

            execute_prediction = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)

            if self.radioButton_9.isChecked():
                self.T_desired = float(self.lineEdit_17.text())
            elif self.radioButton_8.isChecked():
                self.T_desired = 0.0
            try:
                resp = execute_prediction(self.prediction, self.T_desired)

            except AttributeError:
                rospy.loginfo("No refined trajectory available yet!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)   
            
    def on_go_to_click(self):
            
        # reverse predicted trajectory
        if self.radioButton_6.isChecked():
            pred = self.parser.promptraj_msg_to_execution_format(self.prediction)
            pred_rev = self.parser.reverse_trajectory(pred)
            pred_rev_msg = self.parser.predicted_trajectory_to_prompTraj_message(self.parser.point_to_list(self.context))
            try:
                rospy.wait_for_service('execute_trajectory', timeout=2.0)

                execute_reverse_prediction = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
                try:
                    resp = execute_reverse_prediction(pred_rev_msg)
                except AttributeError:
                    rospy.loginfo("No refined trajectory available yet!")
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)   

        # reverse refined trajectory
        elif self.radioButton_7.isChecked():  
            pred = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            pred_rev = self.parser.reverse_trajectory(pred)
            pred_rev_msg = self.parser.predicted_trajectory_to_prompTraj_message(self.parser.point_to_list(self.context))
            try:
                rospy.wait_for_service('execute_trajectory', timeout=2.0)

                execute_reverse_refined = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
                try:
                    resp = execute_reverse_refined(pred_rev_msg)
                    
                except AttributeError:
                    rospy.loginfo("No refined trajectory available yet!")
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)  
        else:
            # get pose from text fields
            pose = Pose()
            try:
                pose.position.x = float(self.lineEdit_9.text())
                pose.position.y = float(self.lineEdit_7.text())
                pose.position.z = float(self.lineEdit_8.text())

                pose.orientation.x = float(self.lineEdit_12.text())
                pose.orientation.y = float(self.lineEdit_11.text())
                pose.orientation.z = float(self.lineEdit_10.text())
                pose.orientation.w = float(self.lineEdit_13.text())
            except ValueError:
                rospy.loginfo("Make sure you set a pose!")

            try:
                rospy.wait_for_service('go_to_pose', timeout=2.0)
                go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
                resp = go_to_pose(pose)

            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)

    def on_add_to_model_click(self):

        try:
            if self.radioButton_2.isChecked():
                rospy.wait_for_service('welford_update', timeout=2.0)

            elif self.radioButton_3.isChecked():
                rospy.wait_for_service('add_demonstration', timeout=2.0)


            rospy.wait_for_service('get_object_position', timeout=2.0)

            reference_frame = String()
            reference_frame.data = 'base'
            get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

            resp = get_object(reference_frame)
            object_wrt_base = resp.object_position
            try:
                refined_trajectory, dt = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            

                refined_trajectory_wrt_object = self.parser.get_trajectory_wrt_context(refined_trajectory, self.parser.point_to_list(object_wrt_base))

                if self.radioButton_2.isChecked():
                    welford_update = rospy.ServiceProxy('welford_update', WelfordUpdate)
                elif self.radioButton_3.isChecked():
                    add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)

                refined_trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(refined_trajectory_wrt_object, self.parser.point_to_list(self.context))
                
                if self.radioButton_2.isChecked():
                    for i in range(int(self.lineEdit_19.text())):
                        resp = welford_update(refined_trajectory_wrt_object_msg)
                    
                    rospy.loginfo("Added " + str(int(self.lineEdit_19.text())) + " trajectories to model using Welford")

                elif self.radioButton_3.isChecked():
                    for i in range(int(self.lineEdit_19.text())):
                        resp = add_demonstration(refined_trajectory_wrt_object_msg)
                    rospy.loginfo("Added " + str(int(self.lineEdit_19.text())) + " trajectories to model using normal computation")
                else:
                    rospy.loginfo("No adding method selected!")

            except (AttributeError, ValueError) as e:
                rospy.loginfo("Problem with adding trajectory: %s" %e)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_calibrate_click(self):
        try:
            rospy.wait_for_service('calibrate_master_pose', timeout=2.0)
            calibrate = rospy.ServiceProxy('calibrate_master_pose', CalibrateMasterPose)
            resp = calibrate()

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_start_teaching_click(self):
        try:
            rospy.wait_for_service('set_teaching_mode', timeout=2.0)
            set_teaching_mode = rospy.ServiceProxy('set_teaching_mode', SetTeachingMode)
            mode = Bool()
            mode.data = True
            set_teaching_mode(mode)

            # rospy.loginfo(("Set teaching mode to {}").format(resp.current_teaching_mode) )

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_stop_teaching_click(self):
        try:
            rospy.wait_for_service('set_teaching_mode', timeout=2.0)
            set_teaching_mode = rospy.ServiceProxy('set_teaching_mode', SetTeachingMode)
            mode = Bool()
            mode.data = False
            set_teaching_mode(mode)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_build_model_click(self):
        if self.radioButton_11.isChecked():
            self.raw_path = self._rospack.get_path('learning_from_demonstration') + "/data/raw/complete_model_situation1/"
        elif self.radioButton_12.isChecked():
            self.raw_path = self._rospack.get_path('learning_from_demonstration') + "/data/raw/complete_model_situation2/"
        elif self.radioButton_13.isChecked():
            self.raw_path = self._rospack.get_path('learning_from_demonstration') + "/data/raw/complete_model_situation3/"

        path = String()
        path.data = self.raw_path

        try:
            rospy.wait_for_service('set_path', timeout=2.0)
            set_path = rospy.ServiceProxy('set_path', SetPath)
            resp = set_path(path)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

        try:
            rospy.wait_for_service('build_initial_model', timeout=2.0)
            build_model = rospy.ServiceProxy('build_initial_model', BuildInitialModel)
            resp = build_model()

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def on_copy_pose_click(self):
        try:
            rospy.wait_for_service('get_ee_pose', timeout=2.0)
            get_ee_pose = rospy.ServiceProxy('get_ee_pose', GetEEPose)
            resp = get_ee_pose()
            
            self.lineEdit_9.setText(str(round(resp.pose.position.x, 3)))
            self.lineEdit_7.setText(str(round(resp.pose.position.y, 3)))
            self.lineEdit_8.setText(str(round(resp.pose.position.z, 3)))

            self.lineEdit_12.setText(str(round(resp.pose.orientation.x, 3)))
            self.lineEdit_11.setText(str(round(resp.pose.orientation.y, 3)))
            self.lineEdit_10.setText(str(round(resp.pose.orientation.z, 3)))
            self.lineEdit_13.setText(str(round(resp.pose.orientation.w, 3)))

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def on_random_object_pose_click(self):
        x = random.uniform(0.7, 0.83)
        y = random.uniform(-0.3, 0.3)
        z = 0.9
        self.lineEdit.setText(str(round(x, 2)))
        self.lineEdit_2.setText(str(round(y, 2)))
        self.lineEdit_3.setText(str(round(z, 2)))

    def on_random_ee_pose_click(self):
        x = random.uniform(0.3, 0.53)
        y = random.uniform(-0.2, 0.4)
        z = random.uniform(0.52, 1.24)

        self.lineEdit_9.setText(str(round(x, 3)))
        self.lineEdit_7.setText(str(round(y, 3)))
        self.lineEdit_8.setText(str(round(z, 3)))

        self.lineEdit_12.setText(str(round(0.980837824843, 3)))
        self.lineEdit_11.setText(str(round(-0.00365989846539, 3)))
        self.lineEdit_10.setText(str(round(-0.194791016723, 3)))
        self.lineEdit_13.setText(str(round(0.000475714270521, 3)))

    def check_button_state(self, button, group_box):


        if group_box.title() == "Movement":

            # if preset 1 and checked
            if button.text() == '1' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.401946359213, 3)))
                self.lineEdit_7.setText(str(round(-0.0230769199229, 3)))
                self.lineEdit_8.setText(str(round(0.840896642238, 3)))

                self.lineEdit_12.setText(str(round(0.980837824843, 3)))
                self.lineEdit_11.setText(str(round(-0.00365989846539, 3)))
                self.lineEdit_10.setText(str(round(-0.194791016723, 3)))
                self.lineEdit_13.setText(str(round(0.000475714270521, 3)))

            elif button.text() == '2' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.371, 3)))
                self.lineEdit_7.setText(str(round(-0.14, 3)))
                self.lineEdit_8.setText(str(round(0.879, 3)))

                self.lineEdit_12.setText(str(round(0.988, 3)))
                self.lineEdit_11.setText(str(round(-0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.151, 3)))
                self.lineEdit_13.setText(str(round(0.03, 3)))
            
            elif button.text() == '3' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
            # elif button.text() == '3' and button.isChecked():
            #     self.lineEdit_9.setText(str(round(0.684, 3)))
            #     self.lineEdit_7.setText(str(round(-0.23, 3)))
            #     self.lineEdit_8.setText(str(round(0.788, 3)))

            #     self.lineEdit_12.setText(str(round(0.988, 3)))
            #     self.lineEdit_11.setText(str(round(-0.0, 3)))
            #     self.lineEdit_10.setText(str(round(-0.123, 3)))
            #     self.lineEdit_13.setText(str(round(0.03, 3)))
        elif group_box.title() == "Environment" and self.radioButton_11.isChecked():

            if button.text() == '1' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '2' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")
            
            elif button.text() == '3' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '4' and button.isChecked():
                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '5' and button.isChecked():
                print("Not needed for this condition!")

            
            elif button.text() == '6' and button.isChecked():
                print("Not needed for this condition!")

        # condition 2
        elif group_box.title() == "Environment" and self.radioButton_12.isChecked():
            if button.text() == '1' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '2' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")
            
            elif button.text() == '3' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("-0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '4' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '5' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")
            
            elif button.text() == '6' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.387, 3)))
                self.lineEdit_7.setText(str(round(-0.15, 3)))
                self.lineEdit_8.setText(str(round(0.870, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(0.0, 3)))
                self.lineEdit_10.setText(str(round(-0.152, 3)))
                self.lineEdit_13.setText(str(round(0.030, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("-0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("10")
                self.lineEdit_16.setText("0.7")

        # condition 3
        elif group_box.title() == "Environment" and self.radioButton_13.isChecked():

            if button.text() == '1' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '2' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.85")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")
            
            elif button.text() == '3' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.3")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '4' and button.isChecked():

                self.lineEdit_9.setText(str(round(0.609, 3)))
                self.lineEdit_7.setText(str(round(-0.306, 3)))
                self.lineEdit_8.setText(str(round(0.816, 3)))

                self.lineEdit_12.setText(str(round(0.985, 3)))
                self.lineEdit_11.setText(str(round(-0.103, 3)))
                self.lineEdit_10.setText(str(round(-0.124, 3)))
                self.lineEdit_13.setText(str(round(0.064, 3)))
                
                # object position
                self.lineEdit.setText("0.65")
                self.lineEdit_2.setText("0.0")
                self.lineEdit_3.setText("0.9")

                # obstacle position
                self.lineEdit_14.setText("0.7")
                self.lineEdit_15.setText("-0.15")
                self.lineEdit_16.setText("0.7")

            elif button.text() == '5' and button.isChecked():

                print("Not needed for this condition!")
            
            elif button.text() == '6' and button.isChecked():

                print("Not needed for this condition!")

    
    def on_initialize_experiment_click(self):
        
        if 'learning_from_demonstration' not in self.nodes:
            self.start_node('learning_from_demonstration', 'learning_from_demonstration.launch')
        else: 
            self.stop_node('learning_from_demonstration')
            self.start_node('learning_from_demonstration', 'learning_from_demonstration.launch')
        if 'trajectory_refinement' not in self.nodes:
            self.start_node('trajectory_refinement', 'trajectory_refinement.launch')        
        
        time.sleep(5)
        self.on_clear_trajectories_click()
        self.on_build_model_click()
        self.on_go_to_click()
        self.on_set_object_position_click()
        self.on_set_obstacle_position_click()
        time.sleep(4)

        self.on_get_context_click()
        
        time.sleep(3)
        self.on_predict_click()
        
        time.sleep(1)
        self.on_visualize_prediction_click()
        
    def on_next_trial_click(self):
        self.on_clear_trajectories_click()
        self.on_add_to_model_click()
        self.on_go_to_click()
        self.on_set_object_position_click()
        self.on_set_obstacle_position_click()
        time.sleep(2)

        self.on_get_context_click()
        time.sleep(2)
        self.on_predict_click()
        time.sleep(1)

        self.on_visualize_prediction_click()

    def store_data(self, *args, **kwargs):

        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/gui/data/experiment/'
        
        if "prediction" in kwargs and "refined" in kwargs:
            file_name = 'predicted_trajectory.csv'
            data = 'x, y, z, qx, qy, qz, qw, t, \n'

            with open(path+file_name, 'w+') as f:
                for i,pose in enumerate(self.prediction.poses):
                    data += "%s, %s, %s, %s, %s, %s, %s, %s \n" % (pose.position.x, pose.position.y, pose.position.z, 
                                                                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                                pose.orientation.w, self.prediction.times[i])
                f.write(data)
            
            file_name = 'refined_trajectory.csv'
            data = 'x, y, z, qx, qy, qz, qw, t, \n'

            with open(path+file_name, 'w+') as f:
                for i,pose in enumerate(self.refined_trajectory.poses):
                    data += "%s, %s, %s, %s, %s, %s, %s, %s \n" % (pose.position.x, pose.position.y, pose.position.z, 
                                                                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                                pose.orientation.w, self.prediction.times[i])
                f.write(data)

        elif "prediction" in kwargs and "refined" not in kwargs:
            file_name = 'predicted_trajectory.csv'
            data = 'x, y, z, qx, qy, qz, qw, t, \n'

            with open(path+file_name, 'w+') as f:
                for i,pose in enumerate(self.prediction.poses):
                    data += "%s, %s, %s, %s, %s, %s, %s, %s \n" % (pose.position.x, pose.position.y, pose.position.z, 
                                                                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                                pose.orientation.w, self.prediction.times[i])
                f.write(data)

        elif "prediction" not in kwargs and "refined" in kwargs:
            file_name = 'refined_trajectory.csv'
            data = 'x, y, z, qx, qy, qz, qw, t, \n'
            
            with open(path+file_name, 'w+') as f:
                for i,pose in enumerate(self.refined_trajectory.poses):
                    data += "%s, %s, %s, %s, %s, %s, %s, %s \n" % (pose.position.x, pose.position.y, pose.position.z, 
                                                                pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                                pose.orientation.w, self.prediction.times[i])
                f.write(data)     
    def get_condition(self):
        if self.radioButton_11.isChecked():
            return 1
        elif self.radioButton_12.isChecked():
            return 2
        elif self.radioButton_12.isChecked():
            return 3
        else: 
            print("No condition selected")
            
    def on_obstacle_hit_click(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        condition_msg = Byte()
        condition_msg.data = self.get_condition()
        try:
            rospy.wait_for_service('increment_obstacles_hit', timeout=2.0)
            increment_obstacle_hit = rospy.ServiceProxy('increment_obstacles_hit', IncrementObstaclesHit)
            
            resp = increment_obstacle_hit(number_msg, condition_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_object_missed_click(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        condition_msg = Byte()
        condition_msg.data = self.get_condition()

        try:
            rospy.wait_for_service('increment_object_missed', timeout=2.0)
            increment_object_missed = rospy.ServiceProxy('increment_object_missed', IncrementObjectMissed)
            
            resp = increment_object_missed(number_msg, condition_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def on_store_data_click(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        condition_msg = Byte()
        condition_msg.data = self.get_condition()
        context_msg = Point()
        context_msg = self.context

        if self.radioButton_21.isChecked():
            self.store_data(predicted=1)
            num_updates_msg = UInt32()
            num_updates_msg.data = int(self.lineEdit_19.text())
            before_after_msg = Bool()

            # 1 = before, 0 = after
            if self.radioButton_22.isChecked():
                before_after_msg.data = 1
            elif self.radioButton_23.isChecked():
                before_after_msg.data = 0
            
            print("msg = " + str(before_after_msg.data))
            try:
                rospy.wait_for_service('set_prediction', timeout=2.0)
                set_prediction = rospy.ServiceProxy('set_prediction', SetPrediction)
                print(before_after_msg)
                resp = set_prediction(number_msg, condition_msg, context_msg, before_after_msg, num_updates_msg)
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)
                
        elif self.radioButton_20.isChecked():
            self.store_data(refined=1)
            try:
                rospy.wait_for_service('add_refinement', timeout=2.0)
                add_refinement = rospy.ServiceProxy('add_refinement', AddRefinement)
                
                resp = add_refinement(number_msg, condition_msg, context_msg)
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)
                



    def on_to_csv_click(self):
        try: 
            rospy.wait_for_service('to_csv')
            to_csv = rospy.ServiceProxy('to_csv', ToCsv)
            number_msg = Byte()
            number_msg.data = int(self.lineEdit_20.text())

            resp = to_csv(number_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_2.setTitle(_translate("MainWindow", " Refinement"))
        self.pushButton_6.setText(_translate("MainWindow", "Add to model"))
        self.pushButton_21.setText(_translate("MainWindow", "Refine prediction"))
        self.pushButton_22.setText(_translate("MainWindow", "Visualize"))
        self.pushButton_23.setText(_translate("MainWindow", "Refine refinement"))
        self.pushButton_10.setText(_translate("MainWindow", "Calibrate init pose"))
        self.radioButton_2.setText(_translate("MainWindow", "Welford"))
        self.radioButton_3.setText(_translate("MainWindow", "Normal"))
        self.pushButton_24.setText(_translate("MainWindow", "Execute refinement"))
        self.groupBox_7.setTitle(_translate("MainWindow", "Execution time"))
        self.radioButton_8.setText(_translate("MainWindow", "Predicted"))
        self.radioButton_9.setText(_translate("MainWindow", "Manual"))
        self.lineEdit_17.setText(_translate("MainWindow", "10"))
        self.lineEdit_17.setPlaceholderText(_translate("MainWindow", "10"))
        self.pushButton_26.setText(_translate("MainWindow", "Load trajectory"))
        self.lineEdit_18.setText(_translate("MainWindow", "refined_trajectory_fast"))
        self.lineEdit_19.setText(_translate("MainWindow", "10"))
        self.lineEdit_19.setPlaceholderText(_translate("MainWindow", "10"))
        self.groupBox_3.setTitle(_translate("MainWindow", "                                                            Learning from Demonstration"))
        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_4.setText(_translate("MainWindow", "y: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.pushButton_5.setText(_translate("MainWindow", "Predict"))
        self.pushButton_8.setText(_translate("MainWindow", "Visualize prediction"))
        self.pushButton_9.setText(_translate("MainWindow", "Clear visualizations"))
        self.pushButton_13.setText(_translate("MainWindow", "Execute prediction"))
        self.label_20.setText(_translate("MainWindow", "Initial teaching"))
        self.pushButton_12.setText(_translate("MainWindow", "Start"))
        self.pushButton_11.setText(_translate("MainWindow", "Stop"))
        self.pushButton_14.setText(_translate("MainWindow", "Build model"))
        self.groupBox.setTitle(_translate("MainWindow", "Simulation environment"))
        self.label.setText(_translate("MainWindow", "x: "))
        self.label_2.setText(_translate("MainWindow", "y: "))
        self.label_3.setText(_translate("MainWindow", "z: "))
        self.pushButton_15.setText(_translate("MainWindow", "Random"))
        self.label_16.setText(_translate("MainWindow", "Object"))
        self.label_21.setText(_translate("MainWindow", "Obstacle"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Movement"))
        self.label_7.setText(_translate("MainWindow", "z: "))
        self.label_8.setText(_translate("MainWindow", "y: "))
        self.label_9.setText(_translate("MainWindow", "x: "))
        self.label_10.setText(_translate("MainWindow", "qz: "))
        self.label_11.setText(_translate("MainWindow", "qy: "))
        self.label_12.setText(_translate("MainWindow", "qx: "))
        self.label_13.setText(_translate("MainWindow", "qw: "))
        self.pushButton_16.setText(_translate("MainWindow", "Random"))
        self.pushButton_17.setText(_translate("MainWindow", "Execute"))
        self.label_14.setText(_translate("MainWindow", "Presets"))
        self.label_15.setText(_translate("MainWindow", "Manual control"))
        self.pushButton_18.setText(_translate("MainWindow", "Enable"))
        self.pushButton_19.setText(_translate("MainWindow", "Disable"))
        self.label_19.setText(_translate("MainWindow", "Head joints"))
        self.pushButton_20.setText(_translate("MainWindow", "Initialize"))
        self.radioButton.setText(_translate("MainWindow", "1"))
        self.radioButton_4.setText(_translate("MainWindow", "2"))
        self.radioButton_5.setText(_translate("MainWindow", "3"))
        self.pushButton_25.setText(_translate("MainWindow", "Copy pose"))
        self.radioButton_6.setText(_translate("MainWindow", "Reverse prediction"))
        self.radioButton_7.setText(_translate("MainWindow", "Reverse refinement"))
        self.radioButton_10.setText(_translate("MainWindow", "4"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Nodes"))
        self.label_17.setText(_translate("MainWindow", "LfD"))
        self.pushButton_3.setText(_translate("MainWindow", "Start"))
        self.pushButton_4.setText(_translate("MainWindow", "Stop"))
        self.label_18.setText(_translate("MainWindow", "Refinement"))
        self.pushButton.setText(_translate("MainWindow", "Start"))
        self.pushButton_2.setText(_translate("MainWindow", "Stop"))
        self.groupBox_8.setTitle(_translate("MainWindow", "Experiment"))
        self.groupBox_9.setTitle(_translate("MainWindow", "Model"))
        self.radioButton_11.setText(_translate("MainWindow", "1"))
        self.radioButton_12.setText(_translate("MainWindow", "2"))
        self.radioButton_13.setText(_translate("MainWindow", "3"))
        self.pushButton_27.setText(_translate("MainWindow", "Start logger"))
        self.pushButton_28.setText(_translate("MainWindow", "Stop logger"))
        self.pushButton_29.setText(_translate("MainWindow", "Initialize"))
        self.pushButton_30.setText(_translate("MainWindow", "Next trial"))
        self.groupBox_10.setTitle(_translate("MainWindow", "Environment"))
        self.radioButton_14.setText(_translate("MainWindow", "1"))
        self.radioButton_15.setText(_translate("MainWindow", "2"))
        self.radioButton_16.setText(_translate("MainWindow", "3"))
        self.radioButton_17.setText(_translate("MainWindow", "4"))
        self.radioButton_18.setText(_translate("MainWindow", "5"))
        self.radioButton_19.setText(_translate("MainWindow", "6"))
        self.pushButton_31.setText(_translate("MainWindow", "Store data"))
        self.lineEdit_20.setText(_translate("MainWindow", "1"))
        self.label_22.setText(_translate("MainWindow", "Participant"))
        self.radioButton_20.setText(_translate("MainWindow", "Refined"))
        self.radioButton_21.setText(_translate("MainWindow", "Predicted"))
        self.pushButton_32.setText(_translate("MainWindow", "Obstacle hit"))
        self.pushButton_33.setText(_translate("MainWindow", "Object missed"))
        self.pushButton_34.setText(_translate("MainWindow", "To csv"))
        self.radioButton_22.setText(_translate("MainWindow", "Before"))
        self.radioButton_23.setText(_translate("MainWindow", "After"))
        self.groupBox_4.setTitle(_translate("MainWindow", "RViz"))
        self.menuOnline_teaching_GUI.setTitle(_translate("MainWindow", "Online teaching GUI"))



        # set object and obstacle when OK is pressed
        self.buttonBox.accepted.connect(self.on_set_object_position_click)
        self.buttonBox.accepted.connect(self.on_set_obstacle_position_click)
        self.buttonBox.rejected.connect(self.on_cancel_position_click)

        self.pushButton_9.clicked.connect(self.on_clear_trajectories_click)
        self.pushButton_8.clicked.connect(self.on_visualize_prediction_click)
        self.pushButton_22.clicked.connect(self.on_visualize_refinement_click)
        self.pushButton_10.clicked.connect(self.on_calibrate_click)
        self.pushButton_14.clicked.connect(self.on_build_model_click)
        self.pushButton_30.clicked.connect(self.on_next_trial_click)

        self.pushButton_7.clicked.connect(self.on_get_context_click)
        self.pushButton_5.clicked.connect(self.on_predict_click)
        self.pushButton_4.clicked.connect(lambda:self.stop_node('learning_from_demonstration.launch'))
        self.pushButton_3.clicked.connect(lambda:self.start_node('learning_from_demonstration', 'learning_from_demonstration.launch'))

        self.pushButton_19.clicked.connect(lambda:self.stop_node('teleop_control.launch'))
        self.pushButton_18.clicked.connect(lambda:self.start_node('teleop_control', 'teleop_control.launch'))
        self.pushButton_25.clicked.connect(self.on_copy_pose_click)

        self.pushButton_27.clicked.connect(lambda:self.stop_node('data_logging.launch'))
        self.pushButton_28.clicked.connect(lambda:self.start_node('data_logger', 'data_logging.launch'))

        self.pushButton_26.clicked.connect(self.on_load_trajectory_click)
        self.pushButton_34.clicked.connect(self.on_to_csv_click)
        self.pushButton_31.clicked.connect(self.on_store_data_click)



        # self.pushButton_12.clicked.connect(lambda:self.start_node('learning_from_demonstration', 'trajectory_teaching.launch'))
        # self.pushButton_11.clicked.connect(lambda:self.stop_node('trajectory_teaching.launch'))
        self.pushButton_12.clicked.connect(self.on_start_teaching_click)        
        self.pushButton_11.clicked.connect(self.on_stop_teaching_click)        
        
        self.pushButton_16.clicked.connect(self.on_random_ee_pose_click)
        self.pushButton_15.clicked.connect(self.on_random_object_pose_click)
        self.pushButton_17.clicked.connect(lambda: self.use_multithread(self.on_go_to_click))
        self.pushButton_13.clicked.connect(lambda: self.use_multithread(self.on_execute_prediction_click))
        self.pushButton_20.clicked.connect(self.on_initialize_head_joints_click)
        self.pushButton.clicked.connect(lambda:self.start_node('trajectory_refinement', 'trajectory_refinement.launch'))
        self.pushButton_2.clicked.connect(lambda:self.stop_node('trajectory_refinement.launch'))
        self.pushButton_24.clicked.connect(lambda: self.use_multithread(self.on_execute_refinement_click))

        self.pushButton_6.clicked.connect(self.on_add_to_model_click)
        self.pushButton_29.clicked.connect(self.on_initialize_experiment_click)
        self.pushButton_33.clicked.connect(self.on_object_missed_click)
        self.pushButton_32.clicked.connect(self.on_obstacle_hit_click)


        self.pushButton_21.clicked.connect(lambda: self.use_multithread(self.on_refine_prediction_click))
        self.pushButton_23.clicked.connect(lambda: self.use_multithread(self.on_refine_refinement_click))


        self.radioButton.toggled.connect(lambda:self.check_button_state(self.radioButton, self.groupBox_6))
        self.radioButton_4.toggled.connect(lambda:self.check_button_state(self.radioButton_4, self.groupBox_6))
        self.radioButton_5.toggled.connect(lambda:self.check_button_state(self.radioButton_5, self.groupBox_6))
        self.radioButton_14.toggled.connect(lambda:self.check_button_state(self.radioButton_14, self.groupBox_10))
        self.radioButton_15.toggled.connect(lambda:self.check_button_state(self.radioButton_15, self.groupBox_10))
        self.radioButton_16.toggled.connect(lambda:self.check_button_state(self.radioButton_16, self.groupBox_10))
        self.radioButton_17.toggled.connect(lambda:self.check_button_state(self.radioButton_17, self.groupBox_10))
        self.radioButton_18.toggled.connect(lambda:self.check_button_state(self.radioButton_18, self.groupBox_10))
        self.radioButton_19.toggled.connect(lambda:self.check_button_state(self.radioButton_19, self.groupBox_10))




if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = experimentGUI()
    
    sys.exit(app.exec_())
