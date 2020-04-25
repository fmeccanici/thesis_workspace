#!/usr/bin/env python

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# other packages
import rospy, sys, roslaunch, rospkg, os, random

from rviz_python.rviz_python import rvizPython

# ROS messages
from learning_from_demonstration.srv import (AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, 
                                            SetObject, SetObjectResponse, GetContext, GetContextResponse, 
                                            GoToPose, GoToPoseResponse, ExecuteTrajectory, ExecuteTrajectoryResponse,
                                            GetObjectPosition, GetObjectPositionResponse, WelfordUpdate, 
                                            WelfordUpdateResponse)
                                            
from trajectory_refinement.srv import RefineTrajectory, RefineTrajectoryResponse, CalibrateMasterPose
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point
from std_msgs.msg import String

from promp_context_ros.msg import prompTraj
from trajectory_visualizer.srv import VisualizeTrajectory, VisualizeTrajectoryResponse, ClearTrajectories, ClearTrajectoriesResponse
from trajectory_visualizer.msg import TrajectoryVisualization

from learning_from_demonstration_python.trajectory_parser import trajectoryParser

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
        self.resize(1680, 1050)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(350, 690, 351, 171))
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
        self.pushButton_6.raise_()
        self.pushButton_21.raise_()
        self.pushButton_22.raise_()
        self.pushButton_23.raise_()
        self.pushButton_10.raise_()
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(620, 690, 521, 291))
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
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(0, 690, 331, 161))
        self.groupBox.setObjectName("groupBox")
        self.label = QLabel(self.groupBox)
        self.label.setGeometry(QRect(18, 26, 16, 17))
        self.label.setObjectName("label")
        self.lineEdit = QLineEdit(self.groupBox)
        self.lineEdit.setGeometry(QRect(39, 26, 146, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setGeometry(QRect(18, 59, 16, 17))
        self.label_2.setObjectName("label_2")
        self.lineEdit_2 = QLineEdit(self.groupBox)
        self.lineEdit_2.setGeometry(QRect(39, 59, 146, 27))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.label_3 = QLabel(self.groupBox)
        self.label_3.setGeometry(QRect(18, 92, 16, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit_3 = QLineEdit(self.groupBox)
        self.lineEdit_3.setGeometry(QRect(39, 92, 146, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.buttonBox = QDialogButtonBox(self.groupBox)
        self.buttonBox.setGeometry(QRect(18, 121, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.pushButton_15 = QPushButton(self.groupBox)
        self.pushButton_15.setGeometry(QRect(200, 121, 85, 27))
        self.pushButton_15.setObjectName("pushButton_15")
        self.buttonBox.raise_()
        self.pushButton_15.raise_()
        self.label.raise_()
        self.lineEdit.raise_()
        self.label_2.raise_()
        self.lineEdit_2.raise_()
        self.label_3.raise_()
        self.lineEdit_3.raise_()
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(10, 10, 1661, 671))
        self.groupBox_4.setObjectName("groupBox_4")
        self.groupBox_6 = QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QRect(1210, 690, 431, 301))
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
        self.buttonBox_2 = QDialogButtonBox(self.groupBox_6)
        self.buttonBox_2.setGeometry(QRect(0, 270, 176, 27))
        self.buttonBox_2.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox_2.setObjectName("buttonBox_2")
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
        self.pushButton_16.setGeometry(QRect(180, 270, 85, 27))
        self.pushButton_16.setObjectName("pushButton_16")
        self.pushButton_17 = QPushButton(self.groupBox_6)
        self.pushButton_17.setGeometry(QRect(180, 240, 85, 27))
        self.pushButton_17.setObjectName("pushButton_17")
        self.label_14 = QLabel(self.groupBox_6)
        self.label_14.setGeometry(QRect(200, 120, 67, 17))
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
        self.checkBox = QCheckBox(self.groupBox_6)
        self.checkBox.setGeometry(QRect(201, 141, 39, 22))
        self.checkBox.setObjectName("checkBox")
        self.checkBox_2 = QCheckBox(self.groupBox_6)
        self.checkBox_2.setGeometry(QRect(201, 169, 39, 22))
        self.checkBox_2.setObjectName("checkBox_2")
        self.checkBox_3 = QCheckBox(self.groupBox_6)
        self.checkBox_3.setGeometry(QRect(201, 197, 39, 22))
        self.checkBox_3.setObjectName("checkBox_3")
        self.groupBox_5 = QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QRect(20, 860, 441, 141))
        self.groupBox_5.setObjectName("groupBox_5")
        self.label_16 = QLabel(self.groupBox_5)
        self.label_16.setGeometry(QRect(0, 20, 161, 17))
        self.label_16.setObjectName("label_16")
        self.pushButton_12 = QPushButton(self.groupBox_5)
        self.pushButton_12.setGeometry(QRect(0, 40, 90, 27))
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_11 = QPushButton(self.groupBox_5)
        self.pushButton_11.setGeometry(QRect(0, 70, 88, 27))
        self.pushButton_11.setObjectName("pushButton_11")
        self.label_17 = QLabel(self.groupBox_5)
        self.label_17.setGeometry(QRect(200, 20, 31, 17))
        self.label_17.setObjectName("label_17")
        self.pushButton_3 = QPushButton(self.groupBox_5)
        self.pushButton_3.setGeometry(QRect(170, 40, 90, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QPushButton(self.groupBox_5)
        self.pushButton_4.setGeometry(QRect(170, 70, 88, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.label_18 = QLabel(self.groupBox_5)
        self.label_18.setGeometry(QRect(330, 20, 91, 17))
        self.label_18.setObjectName("label_18")
        self.pushButton = QPushButton(self.groupBox_5)
        self.pushButton.setGeometry(QRect(330, 40, 90, 27))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QPushButton(self.groupBox_5)
        self.pushButton_2.setGeometry(QRect(330, 73, 88, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1650, 25))
        self.menubar.setObjectName("menubar")
        self.menuOnline_teaching_GUI = QMenu(self.menubar)
        self.menuOnline_teaching_GUI.setObjectName("menuOnline_teaching_GUI")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuOnline_teaching_GUI.menuAction())

        
        self.lineEdit.setText("0.8")
        self.lineEdit_2.setText("0.0")
        self.lineEdit_3.setText("0.9")

        self.rviz_widget = rvizPython()
        self.horizontalLayout_6 = QHBoxLayout()
        self.groupBox_4.setLayout(self.horizontalLayout_6)
        self.horizontalLayout_6.addWidget(self.rviz_widget)
        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)

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


    def stop_node(self, launch_file):
        # look through dictionary to find corresponding launch object
        self.nodes[launch_file].shutdown()

    def start_refinement_node(self):
        package = 'trajectory_refinement'
        launch_file = 'trajectory_refinement.launch'

        abs_path = self._rospack.get_path(package) + "/launch/" + launch_file
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)


        self.refinement_launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])
        self.refinement_launch.start()

        rospy.loginfo( ("Started {} ").format(launch_file) )
    
    def stop_refinement_node(self):
        try:
            self.refinement_launch.shutdown()
        except AttributeError:
            rospy.loginfo( ("Refinement node not launched yet") )

    def start_lfd_node(self):
        package = 'learning_from_demonstration'
        launch_file = 'learning_from_demonstration.launch'

        abs_path = self._rospack.get_path(package) + "/launch/" + launch_file
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)


        self.lfd_launch = roslaunch.parent.ROSLaunchParent(uuid, [abs_path])
        self.lfd_launch.start()

        rospy.loginfo( ("Started {} ").format(launch_file) )

    def stop_lfd_node(self):
        try:
            self.lfd_launch.shutdown()
        except AttributeError:
            rospy.loginfo( ("LfD node not launched yet") )

    def on_refine_prediction_click(self):
        try:
            # rospy.wait_for_service('refine_trajectory', timeout=2.0)

            refine_trajectory = rospy.ServiceProxy('refine_trajectory', RefineTrajectory)
            resp = refine_trajectory(self.prediction)

            self.refined_trajectory = resp.refined_trajectory
            rospy.loginfo("Got a refined trajectory")
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s"%e)

    def on_set_object_position_click(self):
        object_position = Point()
        object_position.x = float(self.lineEdit.text())
        object_position.y = float(self.lineEdit_2.text())
        object_position.z = float(self.lineEdit_3.text())

        try:
            rospy.wait_for_service('set_object', timeout=2.0)

            set_object = rospy.ServiceProxy('set_object', SetObject)
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
    
    def on_execute_prediction_click(self):
        try:
            rospy.wait_for_service('execute_trajectory', timeout=2.0)

            execute_prediction = rospy.ServiceProxy('execute_trajectory', ExecuteTrajectory)
            resp = execute_prediction(self.prediction)

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)   
            
    def on_go_to_click(self):
        
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
            # rospy.wait_for_service('add_demonstration', timeout=2.0)
            rospy.wait_for_service('welford_update', timeout=2.0)

            rospy.wait_for_service('get_object_position', timeout=2.0)

            reference_frame = String()
            reference_frame.data = 'base'
            get_object = rospy.ServiceProxy('get_object_position', GetObjectPosition)

            resp = get_object(reference_frame)
            object_wrt_base = resp.object_position
            try:
                refined_trajectory, dt = self.parser.promptraj_msg_to_execution_format(self.refined_trajectory)
            

                refined_trajectory_wrt_object = self.parser.get_trajectory_wrt_context(refined_trajectory, self.parser.point_to_list(object_wrt_base))

                # add_demonstration = rospy.ServiceProxy('add_demonstration', AddDemonstration)
                welford_update = rospy.ServiceProxy('welford_update', WelfordUpdate)

                refined_trajectory_wrt_object_msg = self.parser.predicted_trajectory_to_prompTraj_message(refined_trajectory_wrt_object, self.parser.point_to_list(self.context))
                # resp = add_demonstration(refined_trajectory_wrt_object_msg)
                resp = welford_update(refined_trajectory_wrt_object_msg)
                
                rospy.loginfo("Added trajectory to model")
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

    def check_button_state(self, button):
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
            self.lineEdit_9.setText(str(round(0.403399335619, 3)))
            self.lineEdit_7.setText(str(round(-0.430007534239, 3)))
            self.lineEdit_8.setText(str(round(1.16269467394, 3)))

            self.lineEdit_12.setText(str(round(0.980837824843, 3)))
            self.lineEdit_11.setText(str(round(-0.00365989846539, 3)))
            self.lineEdit_10.setText(str(round(-0.194791016723, 3)))
            self.lineEdit_13.setText(str(round(0.000475714270521, 3)))
        elif button.text() == '3' and button.isChecked():
            self.lineEdit_9.setText(str(round(0.353543514402, 3)))
            self.lineEdit_7.setText(str(round(0.435045131507, 3)))
            self.lineEdit_8.setText(str(round(0.760080619348, 3)))

            self.lineEdit_12.setText(str(round(0.980837824843, 3)))
            self.lineEdit_11.setText(str(round(-0.00365989846539, 3)))
            self.lineEdit_10.setText(str(round(-0.194791016723, 3)))
            self.lineEdit_13.setText(str(round(0.000475714270521, 3)))

    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_2.setTitle(_translate("MainWindow", " Refinement"))
        self.pushButton_6.setText(_translate("MainWindow", "Add to model"))
        self.pushButton_21.setText(_translate("MainWindow", "Refine prediction"))
        self.pushButton_22.setText(_translate("MainWindow", "Visualize"))
        self.pushButton_23.setText(_translate("MainWindow", "Refine refinement"))
        self.pushButton_10.setText(_translate("MainWindow", "Calibrate init pose"))
        self.groupBox_3.setTitle(_translate("MainWindow", "                                                            Learning from Demonstration"))
        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_4.setText(_translate("MainWindow", "y: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.pushButton_5.setText(_translate("MainWindow", "Predict"))
        self.pushButton_8.setText(_translate("MainWindow", "Visualize prediction"))
        self.pushButton_9.setText(_translate("MainWindow", "Clear visualizations"))
        self.pushButton_13.setText(_translate("MainWindow", "Execute prediction"))
        self.groupBox.setTitle(_translate("MainWindow", "     Set object position Gazebo"))
        self.label.setText(_translate("MainWindow", "x: "))
        self.label_2.setText(_translate("MainWindow", "y: "))
        self.label_3.setText(_translate("MainWindow", "z: "))
        self.pushButton_15.setText(_translate("MainWindow", "Random"))
        self.groupBox_4.setTitle(_translate("MainWindow", "RViz"))
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
        self.checkBox.setText(_translate("MainWindow", "1"))
        self.checkBox_2.setText(_translate("MainWindow", "2"))
        self.checkBox_3.setText(_translate("MainWindow", "3"))
        self.label_14.setText(_translate("MainWindow", "Presets"))
        self.label_15.setText(_translate("MainWindow", "Manual control"))
        self.pushButton_18.setText(_translate("MainWindow", "Enable"))
        self.pushButton_19.setText(_translate("MainWindow", "Disable"))
        self.label_19.setText(_translate("MainWindow", "Head joints"))
        self.pushButton_20.setText(_translate("MainWindow", "Initialize"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Nodes"))
        self.label_16.setText(_translate("MainWindow", "Initial demonstrations"))
        self.pushButton_12.setText(_translate("MainWindow", "Start"))
        self.pushButton_11.setText(_translate("MainWindow", "Stop"))
        self.label_17.setText(_translate("MainWindow", "LfD"))
        self.pushButton_3.setText(_translate("MainWindow", "Start"))
        self.pushButton_4.setText(_translate("MainWindow", "Stop"))
        self.label_18.setText(_translate("MainWindow", "Refinement"))
        self.pushButton.setText(_translate("MainWindow", "Start"))
        self.pushButton_2.setText(_translate("MainWindow", "Stop"))
        self.menuOnline_teaching_GUI.setTitle(_translate("MainWindow", "Online teaching GUI"))

        self.buttonBox.accepted.connect(self.on_set_object_position_click)
        self.pushButton_9.clicked.connect(self.on_clear_trajectories_click)
        self.pushButton_8.clicked.connect(self.on_visualize_prediction_click)
        self.pushButton_22.clicked.connect(self.on_visualize_refinement_click)
        self.pushButton_10.clicked.connect(self.on_calibrate_click)

        self.pushButton_7.clicked.connect(self.on_get_context_click)
        self.pushButton_5.clicked.connect(self.on_predict_click)
        self.pushButton_4.clicked.connect(self.stop_lfd_node)
        self.pushButton_3.clicked.connect(self.start_lfd_node)
        
        self.pushButton_12.clicked.connect(lambda:self.start_node('learning_from_demonstration', 'trajectory_teaching.launch'))
        self.pushButton_11.clicked.connect(lambda:self.stop_node('trajectory_teaching.launch'))
        
        
        self.pushButton_16.clicked.connect(self.on_random_ee_pose_click)
        self.pushButton_15.clicked.connect(self.on_random_object_pose_click)
        self.pushButton_17.clicked.connect(lambda: self.use_multithread(self.on_go_to_click))
        self.pushButton_13.clicked.connect(lambda: self.use_multithread(self.on_execute_prediction_click))
        self.pushButton_20.clicked.connect(self.on_initialize_head_joints_click)
        self.pushButton.clicked.connect(self.start_refinement_node)
        self.pushButton_2.clicked.connect(self.stop_refinement_node)

        self.pushButton_6.clicked.connect(self.on_add_to_model_click)


        self.pushButton_21.clicked.connect(lambda: self.use_multithread(self.on_refine_prediction_click))


        self.checkBox.toggled.connect(lambda:self.check_button_state(self.checkBox))
        self.checkBox_2.toggled.connect(lambda:self.check_button_state(self.checkBox_2))
        self.checkBox_3.toggled.connect(lambda:self.check_button_state(self.checkBox_3))



if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = experimentGUI()
    
    sys.exit(app.exec_())
