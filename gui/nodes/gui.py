#!/usr/bin/env python

# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# other packages
import rospy, sys, roslaunch, rospkg, os
from rviz_python.rviz_python import rvizPython

# ROS messages
from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, SetObject, SetObjectResponse, GetContext, GetContextResponse
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point
from learning_from_demonstration.msg import prompTraj
from trajectory_visualizer.srv import VisualizeTrajectory, VisualizeTrajectoryResponse, ClearTrajectories, ClearTrajectoriesResponse
from trajectory_visualizer.msg import TrajectoryVisualization

class experimentGUI(QMainWindow):

    def __init__(self, *args, **kwargs):

        # inheretance on QMainWindow class
        super(experimentGUI, self).__init__(*args, **kwargs)


        ## initialize ROS stuff
        rospy.init_node("experiment_gui")
        self._rospack = rospkg.RosPack()

        # initialize Qt GUI
        self.initGUI()
        self.show()

    def initGUI(self):
        self.setObjectName("Online teaching GUI")
        self.resize(800, 600)

        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(-10, 10, 221, 162))
        self.groupBox.setObjectName("groupBox")
        self.gridLayout = QGridLayout(self.groupBox)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QLabel(self.groupBox)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.rviz_widget = rvizPython()

        self.rviz_groupbox = QGroupBox(self.rviz_widget)

        self.gridLayout_rviz = QGridLayout(self.rviz_groupbox)
        # self.gridLayout.addWidget(self.rviz_widget, 0, 0, 5, 5)

        # .addWidget(self.rviz, 0, 0 , 2, 2)

        self.lineEdit = QLineEdit(self.groupBox)
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout.addWidget(self.lineEdit, 0, 1, 1, 1)
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.lineEdit_2 = QLineEdit(self.groupBox)
        self.lineEdit_2.setObjectName("lineEdit_2")

        self.gridLayout.addWidget(self.lineEdit_2, 1, 1, 1, 1)
        self.label_3 = QLabel(self.groupBox)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.lineEdit_3 = QLineEdit(self.groupBox)
        self.lineEdit_3.setObjectName("lineEdit_3")

        self.lineEdit.setText("0.8")
        self.lineEdit_2.setText("0.0")
        self.lineEdit_3.setText("0.9")

        self.gridLayout.addWidget(self.lineEdit_3, 2, 1, 1, 1)
        self.buttonBox = QDialogButtonBox(self.groupBox)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 3, 0, 1, 2)
        self.buttonBox.raise_()
        self.label.raise_()
        self.label_2.raise_()
        self.label_3.raise_()
        self.lineEdit.raise_()
        self.lineEdit_2.raise_()
        self.lineEdit_3.raise_()
        self.lineEdit_2.raise_()
        self.lineEdit_3.raise_()
        self.label_2.raise_()
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(10, 400, 272, 96))
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_3 = QGridLayout(self.groupBox_2)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.pushButton_2 = QPushButton(self.groupBox_2)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout_3.addWidget(self.pushButton_2, 1, 0, 1, 1)
        self.pushButton = QPushButton(self.groupBox_2)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_3.addWidget(self.pushButton, 0, 0, 1, 1)
        self.pushButton_6 = QPushButton(self.groupBox_2)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout_3.addWidget(self.pushButton_6, 0, 1, 1, 1)
        self.pushButton.raise_()
        self.pushButton_2.raise_()
        self.pushButton_6.raise_()
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(10, 190, 551, 162))
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout_2 = QGridLayout(self.groupBox_3)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.pushButton_3 = QPushButton(self.groupBox_3)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout_2.addWidget(self.pushButton_3, 0, 0, 1, 1)
        self.pushButton_7 = QPushButton(self.groupBox_3)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout_2.addWidget(self.pushButton_7, 0, 1, 1, 2)
        self.pushButton_5 = QPushButton(self.groupBox_3)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout_2.addWidget(self.pushButton_5, 0, 3, 1, 1)
        self.pushButton_4 = QPushButton(self.groupBox_3)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout_2.addWidget(self.pushButton_4, 1, 0, 1, 1)
        self.label_5 = QLabel(self.groupBox_3)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 1, 1, 1, 1)
        self.lineEdit_5 = QLineEdit(self.groupBox_3)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.gridLayout_2.addWidget(self.lineEdit_5, 1, 2, 1, 1)
        self.pushButton_8 = QPushButton(self.groupBox_3)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout_2.addWidget(self.pushButton_8, 1, 3, 1, 1)
        self.label_4 = QLabel(self.groupBox_3)
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 2, 1, 1, 1)
        self.lineEdit_6 = QLineEdit(self.groupBox_3)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.gridLayout_2.addWidget(self.lineEdit_6, 2, 2, 1, 1)
        self.pushButton_9 = QPushButton(self.groupBox_3)
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout_2.addWidget(self.pushButton_9, 2, 3, 2, 1)
        self.label_6 = QLabel(self.groupBox_3)
        self.label_6.setObjectName("label_6")
        self.gridLayout_2.addWidget(self.label_6, 3, 1, 1, 1)
        self.lineEdit_4 = QLineEdit(self.groupBox_3)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.gridLayout_2.addWidget(self.lineEdit_4, 3, 2, 1, 1)
        self.pushButton_3.raise_()
        self.pushButton_4.raise_()
        self.pushButton_5.raise_()
        self.pushButton_7.raise_()
        self.label_4.raise_()
        self.label_5.raise_()
        self.lineEdit_4.raise_()
        self.label_6.raise_()
        self.lineEdit_5.raise_()
        self.lineEdit_6.raise_()
        self.pushButton_8.raise_()
        self.pushButton_9.raise_()
        self.groupBox.raise_()
        self.groupBox.raise_()
        self.groupBox_2.raise_()
        self.groupBox_3.raise_()
        self.pushButton_6.raise_()
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        self.menuOnline_teaching_GUI = QMenu(self.menubar)
        self.menuOnline_teaching_GUI.setObjectName("menuOnline_teaching_GUI")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuOnline_teaching_GUI.menuAction())

        self.retranslateUi()
        QMetaObject.connectSlotsByName(self)


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


    def set_object_position(self):
        object_position = Point()
        object_position.x = float(self.lineEdit.text())
        object_position.y = float(self.lineEdit_2.text())
        object_position.z = float(self.lineEdit_3.text())

        rospy.wait_for_service('set_object')
        try:
            set_object = rospy.ServiceProxy('set_object', SetObject)
            resp = set_object(object_position)
            return resp.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def clear_trajectories(self):
        rospy.wait_for_service('clear_trajectories')
        try:
            clear_trajectories = rospy.ServiceProxy('clear_trajectories', ClearTrajectories)

            resp = clear_trajectories()
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def visualize_prediction(self):

        # visualize trajectory
        rospy.wait_for_service('visualize_trajectory')
        try:
            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()
            try:
                visualization_msg.pose_array = self.prediction
            
            except AttributeError:
                rospy.loginfo("No prediction made yet!")
            
            visualization_msg.r = 1.0
            visualization_msg.g = 0.0
            visualization_msg.b = 0.0

            resp = visualize_trajectory(visualization_msg)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    
    def get_context(self):
        rospy.wait_for_service('get_context')
        try:
            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context
            self.lineEdit_6.setText(str(round(self.context.x, 2)))
            self.lineEdit_5.setText(str(round(self.context.y, 2)))
            self.lineEdit_4.setText(str(round(self.context.z, 2)))

        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)       

    def predict(self):
        # make prediction using this context
        rospy.wait_for_service('make_prediction')

        try:
            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = resp.prediction.poses

        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

    def generalize(self):
        rospy.loginfo("CHECK")
        # get context 
        rospy.wait_for_service('get_context')
        try:
            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context
            print(self.context)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

        # make prediction using this context
        rospy.wait_for_service('make_prediction')

        try:
            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = resp.prediction.poses

        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

        # visualize trajectory
        rospy.wait_for_service('visualize_trajectory')
        try:
            visualize_trajectory = rospy.ServiceProxy('visualize_trajectory', VisualizeTrajectory)
            visualization_msg = TrajectoryVisualization()
            visualization_msg.pose_array = self.prediction
            visualization_msg.r = 1.0
            visualization_msg.g = 0.0
            visualization_msg.b = 0.0

            resp = visualize_trajectory(visualization_msg)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    
    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Object position Gazebo"))

        self.buttonBox.accepted.connect(self.set_object_position)

        self.label.setText(_translate("MainWindow", "x: "))
        self.label_2.setText(_translate("MainWindow", "y: "))
        self.label_3.setText(_translate("MainWindow", "z: "))

        self.groupBox_2.setTitle(_translate("MainWindow", "Refinement"))

        self.pushButton.setText(_translate("MainWindow", "Start node"))

        self.pushButton_2.setText(_translate("MainWindow", "Stop node"))
        self.pushButton_6.setText(_translate("MainWindow", "Add demonstration"))

        self.groupBox_3.setTitle(_translate("MainWindow", "Learning from Demonstration"))
        self.pushButton_3.setText(_translate("MainWindow", "Start node"))
        self.pushButton_3.clicked.connect(self.start_lfd_node)

        self.pushButton_4.setText(_translate("MainWindow", "Stop node"))
        self.pushButton_4.clicked.connect(self.stop_lfd_node)

        self.pushButton_5.setText(_translate("MainWindow", "Predict"))
        self.pushButton_5.clicked.connect(self.predict)

        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.pushButton_7.clicked.connect(self.get_context)

        self.label_4.setText(_translate("MainWindow", "y: "))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.pushButton_8.setText(_translate("MainWindow", "Visualize prediction"))
        self.pushButton_8.clicked.connect(self.visualize_prediction)

        self.pushButton_9.setText(_translate("MainWindow", "Clear visualizations"))
        self.pushButton_9.clicked.connect(self.clear_trajectories)

        self.menuOnline_teaching_GUI.setTitle(_translate("MainWindow", "Online teaching GUI"))

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = experimentGUI()
    
    sys.exit(app.exec_())
