
# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rospy, rospkg, roslaunch, sys
from learning_from_demonstration_python.trajectory_parser import trajectoryParser


# ros services
from learning_from_demonstration.srv import GoToPose
from gazebo_msgs.srv import SetModelState

# ros messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState 

# rviz python
from rviz_python.rviz_python import rvizPython


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
        
        self.nodes = {}

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
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(680, 660, 411, 171))
        self.groupBox_6.setObjectName("groupBox_6")
        self.pushButton_21 = QPushButton(self.groupBox_6)
        self.pushButton_21.setGeometry(QRect(10, 30, 150, 27))
        self.pushButton_21.setObjectName("pushButton_21")
        self.pushButton_23 = QPushButton(self.groupBox_6)
        self.pushButton_23.setGeometry(QRect(10, 60, 150, 27))
        self.pushButton_23.setObjectName("pushButton_23")
        self.pushButton_22 = QPushButton(self.groupBox_6)
        self.pushButton_22.setGeometry(QRect(10, 90, 150, 27))
        self.pushButton_22.setObjectName("pushButton_22")
        self.checkBox_3 = QCheckBox(self.groupBox_6)
        self.checkBox_3.setGeometry(QRect(10, 120, 97, 22))
        self.checkBox_3.setObjectName("checkBox_3")
        self.checkBox_4 = QCheckBox(self.groupBox_6)
        self.checkBox_4.setGeometry(QRect(10, 150, 97, 22))
        self.checkBox_4.setObjectName("checkBox_4")
        self.pushButton_6 = QPushButton(self.groupBox_6)
        self.pushButton_6.setGeometry(QRect(170, 30, 150, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.checkBox_7 = QCheckBox(self.groupBox_6)
        self.checkBox_7.setGeometry(QRect(110, 120, 97, 22))
        self.checkBox_7.setObjectName("checkBox_7")
        self.groupBox_7 = QGroupBox(self.centralwidget)
        self.groupBox_7.setGeometry(QRect(680, 850, 411, 171))
        self.groupBox_7.setObjectName("groupBox_7")
        self.pushButton_24 = QPushButton(self.groupBox_7)
        self.pushButton_24.setGeometry(QRect(10, 30, 150, 27))
        self.pushButton_24.setObjectName("pushButton_24")
        self.pushButton_25 = QPushButton(self.groupBox_7)
        self.pushButton_25.setGeometry(QRect(10, 60, 150, 27))
        self.pushButton_25.setObjectName("pushButton_25")
        self.pushButton_26 = QPushButton(self.groupBox_7)
        self.pushButton_26.setGeometry(QRect(10, 90, 150, 27))
        self.pushButton_26.setObjectName("pushButton_26")
        self.checkBox_5 = QCheckBox(self.groupBox_7)
        self.checkBox_5.setGeometry(QRect(10, 120, 97, 22))
        self.checkBox_5.setObjectName("checkBox_5")
        self.checkBox_6 = QCheckBox(self.groupBox_7)
        self.checkBox_6.setGeometry(QRect(10, 150, 97, 22))
        self.checkBox_6.setObjectName("checkBox_6")
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
        self.label_18 = QLabel(self.groupBox_12)
        self.label_18.setGeometry(QRect(110, 30, 91, 17))
        self.label_18.setObjectName("label_18")
        self.pushButton_2 = QPushButton(self.groupBox_12)
        self.pushButton_2.setGeometry(QRect(110, 50, 90, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_5 = QPushButton(self.groupBox_12)
        self.pushButton_5.setGeometry(QRect(110, 83, 88, 27))
        self.pushButton_5.setObjectName("pushButton_5")
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
        self.groupBox_6.setTitle(_translate("MainWindow", "Refinement"))
        self.pushButton_21.setText(_translate("MainWindow", "Refine prediction"))
        self.pushButton_23.setText(_translate("MainWindow", "Refine refinement"))
        self.pushButton_22.setText(_translate("MainWindow", "Visualize"))
        self.checkBox_3.setText(_translate("MainWindow", "Refined"))
        self.checkBox_4.setText(_translate("MainWindow", "Predicted"))
        self.pushButton_6.setText(_translate("MainWindow", "Add to model"))
        self.checkBox_7.setText(_translate("MainWindow", "Clear"))
        self.groupBox_7.setTitle(_translate("MainWindow", "LfD"))
        self.pushButton_24.setText(_translate("MainWindow", "Predict"))
        self.pushButton_25.setText(_translate("MainWindow", "Refine refinement"))
        self.pushButton_26.setText(_translate("MainWindow", "Visualize"))
        self.checkBox_5.setText(_translate("MainWindow", "Refined"))
        self.checkBox_6.setText(_translate("MainWindow", "Predicted"))
        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.label_4.setText(_translate("MainWindow", "y: "))
        self.groupBox_12.setTitle(_translate("MainWindow", "Nodes"))
        self.label_17.setText(_translate("MainWindow", "LfD"))
        self.pushButton_3.setText(_translate("MainWindow", "Start"))
        self.pushButton_4.setText(_translate("MainWindow", "Stop"))
        self.label_18.setText(_translate("MainWindow", "Refinement"))
        self.pushButton_2.setText(_translate("MainWindow", "Start"))
        self.pushButton_5.setText(_translate("MainWindow", "Stop"))
        self.pushButton_39.setText(_translate("MainWindow", "Start"))
        self.label_25.setText(_translate("MainWindow", "Keyboard"))
        self.pushButton_38.setText(_translate("MainWindow", "Stop"))
        self.pushButton_43.setText(_translate("MainWindow", "Start"))
        self.pushButton_44.setText(_translate("MainWindow", "Stop"))
        self.label_26.setText(_translate("MainWindow", "Logger"))



        # button events
        self.pushButton_8.clicked.connect(lambda:self.useMultithread(self.onInitializeClick))
        self.pushButton_3.clicked.connect(lambda:self.startNode('learning_from_demonstration', 'learning_from_demonstration.launch'))
        self.pushButton_4.clicked.connect(lambda:self.stopNode('learning_from_demonstration.launch'))
        self.pushButton_2.clicked.connect(lambda:self.startNode('trajectory_refinement', 'trajectory_refinement.launch'))
        self.pushButton_5.clicked.connect(lambda:self.stopNode('trajectory_refinement.launch'))
        self.pushButton_39.clicked.connect(lambda:self.startNode('keyboard_control', 'keyboard_control.launch'))
        self.pushButton_38.clicked.connect(lambda:self.stopNode('keyboard_control.launch'))
        
        self.pushButton_43.clicked.connect(lambda:self.startNode('data_logger', 'data_logging.launch'))
        self.pushButton_44.clicked.connect(lambda:self.stopNode('data_logger.launch'))

        self.pushButton_9.clicked.connect(lambda:self.useMultithread(self.onSetObjectPositionClick))


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
                object_position.pose.position.x = 0.85
                object_position.pose.position.y = 0.3
                object_position.pose.position.z = 0.9
            elif self.radioButton_15.isChecked():
                object_position.pose.position.x = 0.85
                object_position.pose.position.y = 0.0
                object_position.pose.position.z = 0.9
            elif self.radioButton_16.isChecked():
                object_position.pose.position.x = 0.65
                object_position.pose.position.y = 0.3
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

    def onInitializeClick(self):
        # self.stopNode('learning_from_demonstration.launch')
        # self.stopNode('trajectory_refinement_keyboard.launch')
        # self.stopNode('keyboard_control.launch')
        # self.stopNode('data_logging.launch')

        # self.startNode('learning_from_demonstration', 'learning_from_demonstration.launch')
        # self.startNode('trajectory_refinement', 'trajectory_refinement_keyboard.launch')
        # self.startNode('teleop_control', 'keyboard_control.launch')
        # self.startNode('data_logger', 'data_logging.launch')

        self.lineEdit.setText('1')
        
        # initialize lift and head joints
        self.initHeadLiftJoint()

        # go to initial pose
        self.gotToInitialPose()

    def gotToInitialPose(self):
        pose = Pose()
        try:
            pose.position.x = 0.609
            pose.position.y = -0.306
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

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = OnlinePendantGUI()
    
    sys.exit(app.exec_())