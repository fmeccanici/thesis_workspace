#!/usr/bin/env python

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rospy, rospkg, roslaunch, sys, time
from learning_from_demonstration_python.trajectory_parser import trajectoryParser


# ros services
from learning_from_demonstration.srv import (GoToPose, MakePrediction, 
                                                GetContext, GetObjectPosition,
                                                WelfordUpdate, ExecuteTrajectory, 
                                                AddDemonstration)
from gazebo_msgs.srv import SetModelState
from trajectory_visualizer.srv import VisualizeTrajectory, VisualizeTrajectoryResponse, ClearTrajectories, ClearTrajectoriesResponse
from data_logger.srv import (CreateParticipant, AddRefinement,
                                SetPrediction, ToCsv, SetObstaclesHit,
                                SetObjectMissed)
from teach_pendant.srv import GetDemonstrationPendant
from trajectory_refinement.srv import RefineTrajectory, CalibrateMasterPose
from std_srvs.srv import Empty

# ros messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.msg import ModelState 
from promp_context_ros.msg import prompTraj
from trajectory_visualizer.msg import TrajectoryVisualization
from std_msgs.msg import String, Bool, Byte, UInt32, Float32


# rviz python
from rviz_python.rviz_python import rvizPython

# error messages
import genpy

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

class OnlinePendantGUI(QMainWindow):
    def __init__(self, *args, **kwargs):
        
        # inheretance on QMainWindow class
        super(OnlinePendantGUI, self).__init__(*args, **kwargs)

        # multithreading
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
        
        ## initialize ROS stuff
        rospy.init_node("experiment_gui")
        self.rospack = rospkg.RosPack()
        self.parser = trajectoryParser()
        self.lift_goal_pub = rospy.Publisher('/lift_controller_ref', JointState, queue_size=10)
        self.head_goal_pub = rospy.Publisher('/head_controller_ref', JointState, queue_size=10)
        self.stop_timer_service = rospy.Service('stop_timer', Empty, self._stopTimer)


        # variables
        self.nodes = {}
        self.elapsed_time_prev = 0
        self.elapsed_time = 0

        # initialize Qt GUI
        self.initGUI()
        self.show()

    def initGUI(self):
        self.setObjectName("MainWindow")
        self.resize(1920, 1080)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(0, 0, 1881, 641))
        self.groupBox_4.setObjectName("groupBox_4")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(0, 650, 681, 371))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_10 = QGroupBox(self.groupBox)
        self.groupBox_10.setGeometry(QRect(110, 30, 111, 121))
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
        self.groupBox_9 = QGroupBox(self.groupBox)
        self.groupBox_9.setGeometry(QRect(10, 30, 91, 81))
        self.groupBox_9.setObjectName("groupBox_9")
        self.radioButton_11 = QRadioButton(self.groupBox_9)
        self.radioButton_11.setGeometry(QRect(10, 30, 117, 22))
        self.radioButton_11.setObjectName("radioButton_11")
        self.radioButton_12 = QRadioButton(self.groupBox_9)
        self.radioButton_12.setGeometry(QRect(10, 60, 117, 22))
        self.radioButton_12.setObjectName("radioButton_12")
        self.groupBox_11 = QGroupBox(self.groupBox)
        self.groupBox_11.setGeometry(QRect(240, 30, 241, 161))
        self.groupBox_11.setObjectName("groupBox_11")
        self.lineEdit = QLineEdit(self.groupBox_11)
        self.lineEdit.setGeometry(QRect(0, 30, 41, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton = QPushButton(self.groupBox_11)
        self.pushButton.setGeometry(QRect(0, 60, 99, 27))
        self.pushButton.setObjectName("pushButton")
        self.groupBox_3 = QGroupBox(self.groupBox_11)
        self.groupBox_3.setGeometry(QRect(110, 0, 120, 171))
        self.groupBox_3.setObjectName("groupBox_3")
        self.lineEdit_21 = QLineEdit(self.groupBox_3)
        self.lineEdit_21.setGeometry(QRect(0, 30, 61, 27))
        self.lineEdit_21.setObjectName("lineEdit_21")
        self.pushButton_40 = QPushButton(self.groupBox_3)
        self.pushButton_40.setGeometry(QRect(0, 60, 90, 27))
        self.pushButton_40.setObjectName("pushButton_40")
        self.pushButton_41 = QPushButton(self.groupBox_3)
        self.pushButton_41.setGeometry(QRect(0, 90, 90, 27))
        self.pushButton_41.setObjectName("pushButton_41")
        self.pushButton_42 = QPushButton(self.groupBox)
        self.pushButton_42.setGeometry(QRect(440, 90, 90, 27))
        self.pushButton_42.setObjectName("pushButton_42")
        self.groupBox_2 = QGroupBox(self.groupBox)
        self.groupBox_2.setGeometry(QRect(540, 30, 120, 91))
        self.groupBox_2.setObjectName("groupBox_2")
        self.pushButton_33 = QPushButton(self.groupBox_2)
        self.pushButton_33.setGeometry(QRect(0, 60, 101, 27))
        self.pushButton_33.setObjectName("pushButton_33")
        self.pushButton_32 = QPushButton(self.groupBox_2)
        self.pushButton_32.setGeometry(QRect(0, 30, 91, 27))
        self.pushButton_32.setObjectName("pushButton_32")
        self.groupBox_5 = QGroupBox(self.groupBox)
        self.groupBox_5.setGeometry(QRect(10, 180, 271, 151))
        self.groupBox_5.setObjectName("groupBox_5")
        self.pushButton_29 = QPushButton(self.groupBox_5)
        self.pushButton_29.setGeometry(QRect(90, 20, 91, 27))
        self.pushButton_29.setObjectName("pushButton_29")
        self.pushButton_31 = QPushButton(self.groupBox_5)
        self.pushButton_31.setGeometry(QRect(0, 20, 91, 27))
        self.pushButton_31.setObjectName("pushButton_31")
        self.lineEdit_20 = QLineEdit(self.groupBox_5)
        self.lineEdit_20.setGeometry(QRect(80, 60, 31, 27))
        self.lineEdit_20.setObjectName("lineEdit_20")
        self.label_22 = QLabel(self.groupBox_5)
        self.label_22.setGeometry(QRect(0, 60, 91, 20))
        self.label_22.setObjectName("label_22")
        self.checkBox_2 = QCheckBox(self.groupBox_5)
        self.checkBox_2.setGeometry(QRect(10, 130, 97, 22))
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox = QCheckBox(self.groupBox_5)
        self.checkBox.setGeometry(QRect(10, 100, 97, 22))
        self.checkBox.setObjectName("checkBox")
        self.pushButton_30 = QPushButton(self.groupBox_5)
        self.pushButton_30.setGeometry(QRect(180, 20, 91, 27))
        self.pushButton_30.setObjectName("pushButton_30")
        self.groupBox_8 = QGroupBox(self.groupBox)
        self.groupBox_8.setGeometry(QRect(350, 180, 271, 151))
        self.groupBox_8.setObjectName("groupBox_8")
        self.pushButton_34 = QPushButton(self.groupBox_8)
        self.pushButton_34.setGeometry(QRect(0, 20, 91, 27))
        self.pushButton_34.setObjectName("pushButton_34")
        self.pushButton_8 = QPushButton(self.groupBox)
        self.pushButton_8.setGeometry(QRect(100, 0, 99, 27))
        self.pushButton_8.setObjectName("pushButton_8")
        self.pushButton_9 = QPushButton(self.groupBox)
        self.pushButton_9.setGeometry(QRect(120, 150, 99, 27))
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_10 = QPushButton(self.groupBox)
        self.pushButton_10.setGeometry(QRect(10, 150, 99, 27))
        self.pushButton_10.setObjectName("pushButton_10")
        self.pushButton_11 = QPushButton(self.groupBox)
        self.pushButton_11.setGeometry(QRect(200, 0, 99, 27))
        self.pushButton_11.setObjectName("pushButton_11")
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(680, 660, 411, 171))
        self.groupBox_6.setObjectName("groupBox_6")
        self.pushButton_21 = QPushButton(self.groupBox_6)
        self.pushButton_21.setGeometry(QRect(10, 30, 150, 27))
        self.pushButton_21.setObjectName("pushButton_21")
        self.pushButton_6 = QPushButton(self.groupBox_6)
        self.pushButton_6.setGeometry(QRect(170, 30, 150, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.radioButton = QRadioButton(self.groupBox_6)
        self.radioButton.setGeometry(QRect(40, 60, 117, 22))
        self.radioButton.setObjectName("radioButton")
        self.radioButton_2 = QRadioButton(self.groupBox_6)
        self.radioButton_2.setGeometry(QRect(40, 90, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.groupBox_7 = QGroupBox(self.centralwidget)
        self.groupBox_7.setGeometry(QRect(680, 850, 411, 171))
        self.groupBox_7.setObjectName("groupBox_7")
        self.pushButton_24 = QPushButton(self.groupBox_7)
        self.pushButton_24.setGeometry(QRect(10, 30, 150, 27))
        self.pushButton_24.setObjectName("pushButton_24")
        self.pushButton_7 = QPushButton(self.groupBox_7)
        self.pushButton_7.setGeometry(QRect(170, 30, 150, 27))
        self.pushButton_7.setObjectName("pushButton_7")
        self.label_5 = QLabel(self.groupBox_7)
        self.label_5.setGeometry(QRect(179, 67, 16, 17))
        self.label_5.setObjectName("label_5")
        self.label_6 = QLabel(self.groupBox_7)
        self.label_6.setGeometry(QRect(179, 133, 16, 17))
        self.label_6.setObjectName("label_6")
        self.label_4 = QLabel(self.groupBox_7)
        self.label_4.setGeometry(QRect(179, 100, 16, 17))
        self.label_4.setObjectName("label_4")
        self.lineEdit_5 = QLineEdit(self.groupBox_7)
        self.lineEdit_5.setGeometry(QRect(200, 67, 91, 27))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.lineEdit_4 = QLineEdit(self.groupBox_7)
        self.lineEdit_4.setGeometry(QRect(200, 133, 91, 27))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.lineEdit_6 = QLineEdit(self.groupBox_7)
        self.lineEdit_6.setGeometry(QRect(200, 100, 91, 27))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.checkBox_7 = QCheckBox(self.groupBox_7)
        self.checkBox_7.setGeometry(QRect(110, 100, 97, 22))
        self.checkBox_7.setObjectName("checkBox_7")
        self.checkBox_4 = QCheckBox(self.groupBox_7)
        self.checkBox_4.setGeometry(QRect(10, 130, 97, 22))
        self.checkBox_4.setObjectName("checkBox_4")
        self.checkBox_3 = QCheckBox(self.groupBox_7)
        self.checkBox_3.setGeometry(QRect(10, 100, 97, 22))
        self.checkBox_3.setObjectName("checkBox_3")
        self.pushButton_22 = QPushButton(self.groupBox_7)
        self.pushButton_22.setGeometry(QRect(10, 70, 150, 27))
        self.pushButton_22.setObjectName("pushButton_22")
        self.groupBox_12 = QGroupBox(self.centralwidget)
        self.groupBox_12.setGeometry(QRect(1060, 650, 511, 361))
        self.groupBox_12.setObjectName("groupBox_12")
        self.label_17 = QLabel(self.groupBox_12)
        self.label_17.setGeometry(QRect(40, 30, 31, 17))
        self.label_17.setObjectName("label_17")
        self.pushButton_3 = QPushButton(self.groupBox_12)
        self.pushButton_3.setGeometry(QRect(10, 50, 90, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QPushButton(self.groupBox_12)
        self.pushButton_4.setGeometry(QRect(10, 80, 88, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_39 = QPushButton(self.groupBox_12)
        self.pushButton_39.setGeometry(QRect(10, 150, 90, 27))
        self.pushButton_39.setObjectName("pushButton_39")
        self.label_25 = QLabel(self.groupBox_12)
        self.label_25.setGeometry(QRect(10, 130, 91, 17))
        self.label_25.setObjectName("label_25")
        self.pushButton_38 = QPushButton(self.groupBox_12)
        self.pushButton_38.setGeometry(QRect(10, 180, 88, 27))
        self.pushButton_38.setObjectName("pushButton_38")
        self.pushButton_43 = QPushButton(self.groupBox_12)
        self.pushButton_43.setGeometry(QRect(120, 150, 90, 27))
        self.pushButton_43.setObjectName("pushButton_43")
        self.pushButton_44 = QPushButton(self.groupBox_12)
        self.pushButton_44.setGeometry(QRect(120, 180, 88, 27))
        self.pushButton_44.setObjectName("pushButton_44")
        self.label_26 = QLabel(self.groupBox_12)
        self.label_26.setGeometry(QRect(120, 130, 91, 17))
        self.label_26.setObjectName("label_26")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1920, 25))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        # initialize default radio buttons
        self.radioButton_11.setChecked(True)
        self.radioButton_14.setChecked(True)
        self.checkBox_2.setChecked(True)
        self.checkBox_4.setChecked(True)
        self.radioButton.setChecked(True)

        # Rviz python 
        config_file = self.rospack.get_path('gui') + "/gui.rviz"

        self.rviz_widget = rvizPython(config_file)
        self.horizontalLayout_6 = QHBoxLayout()
        self.groupBox_4.setLayout(self.horizontalLayout_6)
        self.horizontalLayout_6.addWidget(self.rviz_widget)

        self.retranslateUi(self)
        QMetaObject.connectSlotsByName(self)

    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_4.setTitle(_translate("MainWindow", "RViz"))
        self.groupBox.setTitle(_translate("MainWindow", "Experiment"))
        self.groupBox_10.setTitle(_translate("MainWindow", "Object position"))
        self.radioButton_14.setText(_translate("MainWindow", "1"))
        self.radioButton_15.setText(_translate("MainWindow", "2"))
        self.radioButton_16.setText(_translate("MainWindow", "3"))
        self.radioButton_17.setText(_translate("MainWindow", "4"))
        self.radioButton_18.setText(_translate("MainWindow", "5"))
        self.radioButton_19.setText(_translate("MainWindow", "6"))
        self.groupBox_9.setTitle(_translate("MainWindow", "Variation"))
        self.radioButton_11.setText(_translate("MainWindow", "1"))
        self.radioButton_12.setText(_translate("MainWindow", "2"))
        self.groupBox_11.setTitle(_translate("MainWindow", "Trial"))
        self.pushButton.setText(_translate("MainWindow", "Next"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Timer"))
        self.lineEdit_21.setText(_translate("MainWindow", "0"))
        self.pushButton_40.setText(_translate("MainWindow", "Start"))
        self.pushButton_41.setText(_translate("MainWindow", "Stop"))
        self.pushButton_42.setText(_translate("MainWindow", "Zero"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Failure"))
        self.pushButton_33.setText(_translate("MainWindow", "Object missed"))
        self.pushButton_32.setText(_translate("MainWindow", "Obstacle hit"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Data"))
        self.pushButton_29.setText(_translate("MainWindow", "Load"))
        self.pushButton_31.setText(_translate("MainWindow", "Store"))
        self.lineEdit_20.setText(_translate("MainWindow", "1"))
        self.label_22.setText(_translate("MainWindow", "Participant"))
        self.checkBox_2.setText(_translate("MainWindow", "Predicted"))
        self.checkBox.setText(_translate("MainWindow", "Refined"))
        self.pushButton_30.setText(_translate("MainWindow", "Save"))
        self.groupBox_8.setTitle(_translate("MainWindow", "Movement"))
        self.pushButton_34.setText(_translate("MainWindow", "Initial pose"))
        self.pushButton_8.setText(_translate("MainWindow", "Initialize"))
        self.pushButton_9.setText(_translate("MainWindow", "Set"))
        self.pushButton_10.setText(_translate("MainWindow", "Set"))
        self.pushButton_11.setText(_translate("MainWindow", "Exit"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Refinement"))
        self.pushButton_21.setText(_translate("MainWindow", "Refine"))
        self.pushButton_6.setText(_translate("MainWindow", "Add to model"))
        self.radioButton.setText(_translate("MainWindow", "Prediction"))
        self.radioButton_2.setText(_translate("MainWindow", "Refinement"))
        self.groupBox_7.setTitle(_translate("MainWindow", "LfD"))
        self.pushButton_24.setText(_translate("MainWindow", "Predict"))
        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.label_4.setText(_translate("MainWindow", "y: "))
        self.checkBox_7.setText(_translate("MainWindow", "Clear"))
        self.checkBox_4.setText(_translate("MainWindow", "Predicted"))
        self.checkBox_3.setText(_translate("MainWindow", "Refined"))
        self.pushButton_22.setText(_translate("MainWindow", "Visualize"))
        self.groupBox_12.setTitle(_translate("MainWindow", "Nodes"))
        self.label_17.setText(_translate("MainWindow", "LfD"))
        self.pushButton_3.setText(_translate("MainWindow", "Start"))
        self.pushButton_4.setText(_translate("MainWindow", "Stop"))
        self.pushButton_39.setText(_translate("MainWindow", "Start"))
        self.label_25.setText(_translate("MainWindow", "Keyboard"))
        self.pushButton_38.setText(_translate("MainWindow", "Stop"))
        self.pushButton_43.setText(_translate("MainWindow", "Start"))
        self.pushButton_44.setText(_translate("MainWindow", "Stop"))
        self.label_26.setText(_translate("MainWindow", "Logger"))


        # button events

        # multithreading this function results in segmentation fault
        # self.pushButton_8.clicked.connect(lambda:self.useMultithread(self.onInitializeClick))
        self.pushButton_8.clicked.connect(self.onInitializeClick)

        self.pushButton_3.clicked.connect(lambda:self.startNode('learning_from_demonstration', 'learning_from_demonstration.launch'))
        self.pushButton_4.clicked.connect(lambda:self.stopNode('learning_from_demonstration.launch'))
        self.pushButton_39.clicked.connect(lambda:self.startNode('teach_pendant', 'teach_pendant.launch'))
        self.pushButton_38.clicked.connect(lambda:self.stopNode('teach_pendant.launch'))

        
        # self.pushButton_38.clicked.connect(lambda:self.stopNode('keyboard_control.launch'))
        
        self.pushButton_43.clicked.connect(lambda:self.startNode('data_logger', 'data_logging.launch'))
        self.pushButton_44.clicked.connect(lambda:self.stopNode('data_logger.launch'))

        self.pushButton_9.clicked.connect(lambda:self.useMultithread(self.onSetObjectPositionClick))
        self.pushButton_40.clicked.connect(lambda:self.useMultithread(self.onStartTimerClick))
        self.pushButton_41.clicked.connect(self.onStopTimerClick)
        self.pushButton_42.clicked.connect(self.onZeroTimerClick)
        self.pushButton_22.clicked.connect(self.onVisualizeClick)
        self.pushButton_24.clicked.connect(self.onPredictClick)
        self.pushButton_7.clicked.connect(self.onGetContextClick)

        self.pushButton_34.clicked.connect(lambda:self.useMultithread(self.onInitialPoseClick))
        self.pushButton_21.clicked.connect(lambda:self.useMultithread(self.onRefineClick))
        self.pushButton_6.clicked.connect(self.onAddModelClick)
        self.pushButton_31.clicked.connect(self.onStoreClick)
        self.pushButton_29.clicked.connect(self.onLoadClick)
        self.pushButton_30.clicked.connect(self.onSaveClick)
        self.pushButton.clicked.connect(self.onNextClick)
        self.pushButton_11.clicked.connect(self.onExitClick)

        self.pushButton_32.clicked.connect(self.onObstacleHitClick)
        self.pushButton_33.clicked.connect(self.onObjectMissedClick)






    # multithread for executing trajectories
    # needed since otherwise the GUI will freeze
    def useMultithread(self, function):
        worker = Worker(function)
        self.threadpool.start(worker)

    def startNode(self, package, launch_file):
        
        if launch_file not in self.nodes: 
            abs_path = self.rospack.get_path(package) + "/launch/" + launch_file
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
            abs_path = self.rospack.get_path(package) + "/launch/" + launch_file
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])

            # needed to be able to stop the node
            self.nodes[launch_file] = launch

            self.nodes[launch_file].start()
            rospy.loginfo( ("Started {} ").format(launch_file) )


    def stopNode(self, launch_file):
        # look through dictionary to find corresponding launch object
        try:
            self.nodes[launch_file].shutdown()
        except (AttributeError, KeyError):
            rospy.loginfo( ("Node not launched yet") )

    def startTimer(self):
        self.start_time = time.time()
        # self.stop_timer = False

        # while True:
        #     if self.stop_timer == True:
        #         break
        #     else:
        #         self.elapsed_time = self.elapsed_time_prev + (time.time() - t)
        #         self.lineEdit_21.setText(str(round(self.elapsed_time, 1)))

    def stopTimer(self):
        # # prev is used to store the previous elapsed time
        # self.elapsed_time_prev = self.elapsed_time
        # self.stop_timer = True
        rospy.loginfo("time = " + str(self.elapsed_time))
        self.elapsed_time = self.elapsed_time_prev + (time.time() - self.start_time)
        self.lineEdit_21.setText(str(round(self.elapsed_time, 1)))
        self.elapsed_time_prev = self.elapsed_time

    def zeroTimer(self):
        self.elapsed_time = 0
        self.elapsed_time_prev = 0
        self.lineEdit_21.setText('0')

    def onStartTimerClick(self):
        self.startTimer()

    def onStopTimerClick(self):
        self.stopTimer()

    def onZeroTimerClick(self):
        self.zeroTimer()

    def onPredictClick(self):
        try:
            rospy.wait_for_service('make_prediction', timeout=2.0)

            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = resp.prediction
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
        
        except AttributeError:
            rospy.loginfo("Context not yet extracted!")
    
    def onRefineClick(self):

        try:
            rospy.wait_for_service('execute_trajectory', timeout=2.0)
            execute_trajectory = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)

            self.T_desired = 10.0
            
            # start timer
            self.startTimer()
            

            if self.radioButton.isChecked():
                resp = execute_trajectory(self.prediction, self.T_desired)

            elif self.radioButton_2.isChecked():
                try:
                    rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)

                    get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)

                    resp = get_demo_pendant()
                    self.refined_trajectory = resp.demo
                
                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s" %e)

                resp = execute_trajectory(self.refined_trajectory, self.T_desired)
                
            # self.refined_trajectory = resp.refined_trajectory



            # self.stopTimer()
            # rospy.loginfo("Got a refined trajectory")


            # # make sure the previous refinement is deleted
            # # visualize refinement and prediction
            # self.setVisualizationCheckBoxes(what_to_visualize='clear')
            # self.onVisualizeClick()
            # self.setVisualizationCheckBoxes(what_to_visualize='both')
            # self.onVisualizeClick()
            
            # self.onInitialPoseClick()
            # self.onSetObjectPositionClick()


        except AttributeError as e:
                rospy.loginfo(("No refined trajectory available yet!: {}").format(e))

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)
    
    def _stopTimer(self, req):
        print('check stop timer')
        self.stopTimer()
        resp = Empty()
        return resp
        
    def onAddModelClick(self):

        # 2 was too much, later trajectories had too little influence
        amount = 1

        try:
            rospy.wait_for_service('add_demonstration', timeout=2.0)
            rospy.wait_for_service('get_object_position', timeout=2.0)

            reference_frame = String()
            reference_frame.data = 'base'
            get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

            resp = get_object(reference_frame)
            object_wrt_base = resp.object_position

            refined_trajectory, dt = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            refined_trajectory_wrt_object = self.parser.get_trajectory_wrt_context(refined_trajectory, self.parser.point_to_list(object_wrt_base))

            add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
            refined_trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(refined_trajectory_wrt_object, self.parser.point_to_list(self.context))
            
            for i in range(amount):
                resp = add_demonstration(refined_trajectory_wrt_object_msg)
            
            rospy.loginfo("Added " + str(amount) + " trajectories to model using Welford")
            
            # stop and reset timer
            self.stopTimer()

        except (AttributeError, ValueError) as e:
                rospy.loginfo("Problem with adding trajectory: %s" %e)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def onGetContextClick(self):
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

    def objectPositionToRadioButton(self, object_position):
        if object_position == 1:
            return 14
        elif object_position == 2:
            return 15
        elif object_position == 3:
            return 16
        elif object_position == 4:
            return 17
        elif object_position == 5:
            return 18
        elif object_position == 6:
            return 19

    def onNextClick(self):
        
        self.checkBox.setChecked(True)
        self.checkBox_2.setChecked(True)

        self.onLoadClick()
        self.onStoreClick()
        self.onSaveClick()
        
        self.zeroTimer()

        # move to next trial
        next_trial = int(self.lineEdit.text()) + 1
        max_trials = 5

        # move to next object position if total amount of trials is reached
        if next_trial == max_trials + 1:
            self.lineEdit.setText(str(1))
            next_object_position = self.getObjectPosition() + 1
            exec("self.radioButton_" + str(self.objectPositionToRadioButton(next_object_position)) + ".setChecked(True)")
            
            # move ee to initial pose
            self.onInitialPoseClick()

            # wait until arm is not in the way of the object
            time.sleep(2)
            self.onSetObjectPositionClick()

            # needed to get correct context --> wait until arm is at initial pose
            time.sleep(2)
            
            # get new context
            self.onGetContextClick()
            self.onPredictClick()

            # clear all trajectories
            self.checkBox_3.setChecked(False)
            self.checkBox_4.setChecked(False)
            self.onVisualizeClick()

            # visualize prediction
            self.checkBox_4.setChecked(True)
            self.onVisualizeClick()

        else:
            self.lineEdit.setText(str(next_trial))

            self.onSetObjectPositionClick()

            # move ee to initial pose
            self.onInitialPoseClick()

            # add current refinement to model and make new prediction
            self.onAddModelClick()
            self.onGetContextClick()
            self.onPredictClick()

            # clear all trajectories
            self.checkBox_3.setChecked(False)
            self.checkBox_4.setChecked(False)
            self.onVisualizeClick()

            # visualize prediction
            self.checkBox_4.setChecked(True)
            self.onVisualizeClick()


    def onVisualizeClick(self):
        
        try:
            rospy.wait_for_service('visualize_trajectory', timeout=2.0)

            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()


            if self.checkBox_3.isChecked() and self.checkBox_4.isChecked():
                
                # get demonstration from pendant
                try:
                    rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)

                    get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)

                    resp = get_demo_pendant()
                    self.refined_trajectory = resp.demo
                    rospy.loginfo(self.refined_trajectory)
                    rospy.loginfo("Got an offline taught trajectory using the Keyboard")

                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s"%e)

                visualization_msg.pose_array = self.refined_trajectory.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0
                resp = visualize_trajectory(visualization_msg)

                visualization_msg.pose_array = self.refined_trajectory.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)
                
                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0

                resp = visualize_trajectory(visualization_msg)

            elif self.checkBox_3.isChecked() and not self.checkBox_4.isChecked():
                # get demonstration from pendant

                try:
                    rospy.wait_for_service('get_demonstration_pendant', timeout=2.0)

                    get_demo_pendant = rospy.ServiceProxy('get_demonstration_pendant', GetDemonstrationPendant)

                    resp = get_demo_pendant()
                    self.refined_trajectory = resp.demo
                    rospy.loginfo(self.refined_trajectory)
                    rospy.loginfo("Got an offline taught trajectory using the Keyboard")

                except (rospy.ServiceException, rospy.ROSException) as e:
                    print("Service call failed: %s"%e)

                visualization_msg.pose_array = self.refined_trajectory.poses
                visualization_msg.r = 0.0
                visualization_msg.g = 1.0
                visualization_msg.b = 0.0
                resp = visualize_trajectory(visualization_msg)
            
            elif self.checkBox_4.isChecked() and not self.checkBox_3.isChecked():
                visualization_msg.pose_array = self.prediction.poses
                visualization_msg.r = 1.0
                visualization_msg.g = 0.0
                visualization_msg.b = 0.0                
                resp = visualize_trajectory(visualization_msg)
            elif not self.checkBox_4.isChecked() and not self.checkBox_3.isChecked():
                rospy.wait_for_service('clear_trajectories', timeout=2.0)

                clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)

                resp = clear_trajectories()
            else:
                rospy.loginfo("Visualization combination not possible!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

        except AttributeError:
            rospy.loginfo("No prediction made yet!")

    def initHeadLiftJoint(self):
        lift_goal = JointState()
        head_goal = JointState()

        lift_goal.name = ["torso_lift_joint"]
        head_goal.name = ["head_2_joint", "head_1_joint"]

        # lift_goal.position = [0.19432052791416207]
        lift_goal.position = [0.3]

        head_goal.position = [-0.6499237225775083, 0.0]

        head_goal.effort = [0.0, 0.0]
        lift_goal.effort = [0.0]

        head_goal.velocity = [0.0, 0.0]
        lift_goal.velocity = [0.0]

        rospy.loginfo(lift_goal)

        self.lift_goal_pub.publish(lift_goal)
        self.head_goal_pub.publish(head_goal)

    def onSetObjectPositionClick(self):
        try:
            object_position = ModelState()
            object_position.model_name = 'aruco_cube'

            if self.radioButton_14.isChecked():
                object_position.pose.position.x = 0.82
                object_position.pose.position.y = 0.3
                object_position.pose.position.z = 0.9
            elif self.radioButton_15.isChecked():
                object_position.pose.position.x = 0.82
                object_position.pose.position.y = 0.0
                object_position.pose.position.z = 0.9
            elif self.radioButton_16.isChecked():
                object_position.pose.position.x = 0.65

                # 0.29 instead of 0.30 because otherwise object is on the edge of not
                # being detected
                object_position.pose.position.y = 0.29
                object_position.pose.position.z = 0.9
            elif self.radioButton_17.isChecked():
                object_position.pose.position.x = 0.65
                object_position.pose.position.y = 0.0
                object_position.pose.position.z = 0.9

            object_position.pose.orientation.x = 0
            object_position.pose.orientation.y = 0
            object_position.pose.orientation.z = 0
            object_position.pose.orientation.w = 1
            
            rospy.wait_for_service('/gazebo/set_model_state')

            set_object = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_object(object_position)
            return resp.success

        except ValueError:
            rospy.loginfo("Invalid value for position!")

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def onExitClick(self):
        # stop all nodes before exiting to prevent freezing
        self.stopNode('learning_from_demonstration.launch')
        self.stopNode('trajectory_refinement_keyboard.launch')
        self.stopNode('keyboard_control.launch')
        self.stopNode('data_logging.launch')

    def setVisualizationCheckBoxes(self, what_to_visualize='prediction'):
        if what_to_visualize == 'prediction':
            self.checkBox_4.setChecked(True)
            self.checkBox_3.setChecked(False)

        elif what_to_visualize == 'refinement':
            self.checkBox_4.setChecked(False)
            self.checkBox_3.setChecked(True)

        elif what_to_visualize == 'clear':
            self.checkBox_4.setChecked(False)
            self.checkBox_3.setChecked(False)

        elif what_to_visualize == 'both':
            self.checkBox_4.setChecked(True)
            self.checkBox_3.setChecked(True)

    def onInitializeClick(self):
        self.stopNode('learning_from_demonstration.launch')
        self.stopNode('teach_pendant.launch')
        self.stopNode('data_logging.launch')

        self.startNode('learning_from_demonstration', 'learning_from_demonstration.launch')
        self.startNode('teach_pendant', 'teach_pendant.launch')
        self.startNode('data_logger', 'data_logging.launch')

        self.lineEdit.setText('1')
        
        time.sleep(20)

        # initialize lift and head joints
        self.initHeadLiftJoint()

        # go to initial pose
        self.goToInitialPose()

        self.onSetObjectPositionClick()
        time.sleep(2)
        self.onGetContextClick()

        self.onPredictClick()

        self.setVisualizationCheckBoxes(what_to_visualize='prediction')
        self.onVisualizeClick()


    def onInitialPoseClick(self):
        self.goToInitialPose()

    def goToInitialPose(self):
        pose = Pose()
        try:
            pose.position.x = 0.609
            # pose.position.y = -0.306
            pose.position.y = -0.290


            pose.position.z = 0.816

            pose.orientation.x = 0.985
            pose.orientation.y = -0.103
            pose.orientation.z = -0.124
            pose.orientation.w = 0.064
            
            rospy.wait_for_service('go_to_pose', timeout=2.0)
            go_to_pose = rospy.ServiceProxy('go_to_pose', GoToPose)
            resp = go_to_pose(pose)
        
        except ValueError:
            rospy.loginfo("Make sure you set a pose!")
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def getObjectPosition(self):
        if self.radioButton_14.isChecked():
            return 1
        elif self.radioButton_15.isChecked():
            return 2
        elif self.radioButton_16.isChecked():
            return 3
        elif self.radioButton_17.isChecked():
            return 4
        elif self.radioButton_18.isChecked():
            return 5
        elif self.radioButton_19.isChecked():
            return 6
        else: 
            print("No environment selected")
    
    def getVariation(self):
        if self.radioButton_11.isChecked():
            return 1
        elif self.radioButton_12.isChecked():
            return 2
        else: 
            print("No variation selected")
    
    def getTrial(self):  
        try:
            return int(self.lineEdit.text())
        except ValueError:
            rospy.loginfo("No trial set!")

    def storeData(self, *args, **kwargs):

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
                                                                pose.orientation.w, self.refined_trajectory.times[i])
                f.write(data)

    def onObstacleHitClick(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        method_msg = Byte()

        # method 4: offline + pendant
        method_msg.data = 4
        object_position_msg = Byte()
        object_position_msg.data = self.getObjectPosition()
        context_msg = Point()
        try:
            context_msg = self.context
        except AttributeError:
            rospy.loginfo("Context not set!")
            return -1
        variation_msg = Byte()
        variation_msg.data = self.getVariation()
        trial_msg = Byte()
        trial_msg.data = self.getTrial()
        
        try:
            rospy.wait_for_service('set_obstacles_hit', timeout=2.0)
            set_obstacle_hit = rospy.ServiceProxy('set_obstacles_hit', SetObstaclesHit)
            
            resp = set_obstacle_hit(number_msg, object_position_msg, 
                                    variation_msg, trial_msg)
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def onObjectMissedClick(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        method_msg = Byte()

        # method 4: online + pendant
        method_msg.data = 4
        object_position_msg = Byte()
        object_position_msg.data = self.getObjectPosition()
        context_msg = Point()
        try:
            context_msg = self.context
        except AttributeError:
            rospy.loginfo("Context not set!")
            return -1
        variation_msg = Byte()
        variation_msg.data = self.getVariation()
        trial_msg = Byte()
        trial_msg.data = self.getTrial()

        try:
            rospy.wait_for_service('set_object_missed', timeout=2.0)
            set_object_missed = rospy.ServiceProxy('set_object_missed', SetObjectMissed)
            
            resp = set_object_missed(number_msg, object_position_msg, 
                                    variation_msg, trial_msg)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def onStoreClick(self):
        number_msg = Byte()
        number_msg.data = int(self.lineEdit_20.text())
        method_msg = Byte()

        # method 4: online + pendant
        method_msg.data = 4
        object_position_msg = Byte()
        object_position_msg.data = self.getObjectPosition()
        context_msg = Point()
        try:
            context_msg = self.context
        except AttributeError:
            rospy.loginfo("Context not set!")
            return -1

        variation_msg = Byte()
        variation_msg.data = self.getVariation()
        trial_msg = Byte()
        trial_msg.data = self.getTrial()
            
        time_msg = Float32()
        time_msg.data = self.elapsed_time

    
        if self.checkBox.isChecked() and self.checkBox_2.isChecked():
            self.storeData(predicted=1)   
            self.storeData(refined=1)
         
            try:
                rospy.wait_for_service('set_prediction', timeout=2.0)
                set_prediction = rospy.ServiceProxy('set_prediction', SetPrediction)

                resp = set_prediction(number_msg, object_position_msg, variation_msg,
                                    trial_msg, context_msg, time_msg, method_msg)

                
                rospy.wait_for_service('add_refinement', timeout=2.0)
                add_refinement = rospy.ServiceProxy('add_refinement', AddRefinement)
                resp = add_refinement(number_msg, object_position_msg, variation_msg,
                                    trial_msg, context_msg, time_msg, method_msg)
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)
        
        elif self.checkBox.isChecked() and not self.checkBox_2.isChecked():
            self.storeData(refined=1)
            try:
                rospy.wait_for_service('add_refinement', timeout=2.0)
                add_refinement = rospy.ServiceProxy('add_refinement', AddRefinement)
                resp = add_refinement(number_msg, object_position_msg, variation_msg,
                                    trial_msg, context_msg, time_msg, method_msg)
            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call failed: %s" %e)
        
        elif not self.checkBox.isChecked() and self.checkBox_2.isChecked():
            self.storeData(predicted=1)
            try:
                rospy.wait_for_service('set_prediction', timeout=2.0)
                set_prediction = rospy.ServiceProxy('set_prediction', SetPrediction)

                resp = set_prediction(number_msg, object_position_msg, variation_msg,
                                    trial_msg, context_msg, time_msg, method_msg)

            except (rospy.ServiceException, rospy.ROSException, genpy.message.SerializationError) as e:
                print("Service call failed: %s" %e)
        else:
            rospy.loginfo("No trajectory selected for storage!")
    
    def onSaveClick(self):
        try: 
            rospy.wait_for_service('to_csv')
            to_csv = rospy.ServiceProxy('to_csv', ToCsv)
            number_msg = Byte()
            number_msg.data = int(self.lineEdit_20.text())

            resp = to_csv(number_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def onLoadClick(self):
        try: 
            rospy.wait_for_service('create_participant', timeout=2.0)
            create_participant = rospy.ServiceProxy('create_participant', CreateParticipant)
            number_msg = Byte()

            number_msg.data = int(self.lineEdit_20.text())

            # dummy variable names --> not necessary when data exists

            sex_msg = Bool()
            sex_msg.data = 1
            age_msg = Byte()
            age_msg.data = 1

            resp = create_participant(number_msg, sex_msg, age_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = OnlinePendantGUI()
    
    sys.exit(app.exec_())