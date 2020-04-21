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
        self.resize(1680, 1050)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(660, 700, 161, 159))
        self.groupBox_2.setObjectName("groupBox_2")
        self.pushButton_2 = QPushButton(self.groupBox_2)
        self.pushButton_2.setGeometry(QRect(1, 64, 88, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton = QPushButton(self.groupBox_2)
        self.pushButton.setGeometry(QRect(1, 31, 90, 27))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_6 = QPushButton(self.groupBox_2)
        self.pushButton_6.setGeometry(QRect(1, 97, 150, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(820, 700, 491, 159))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton_7 = QPushButton(self.groupBox_3)
        self.pushButton_7.setGeometry(QRect(150, 20, 99, 27))
        self.pushButton_7.setObjectName("pushButton_7")
        self.pushButton_4 = QPushButton(self.groupBox_3)
        self.pushButton_4.setGeometry(QRect(18, 59, 88, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_3 = QPushButton(self.groupBox_3)
        self.pushButton_3.setGeometry(QRect(18, 26, 90, 27))
        self.pushButton_3.setObjectName("pushButton_3")
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
        self.pushButton_9.setGeometry(QRect(340, 92, 148, 27))
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_4.raise_()
        self.pushButton_3.raise_()
        self.label_5.raise_()
        self.lineEdit_5.raise_()
        self.label_4.raise_()
        self.lineEdit_6.raise_()
        self.label_6.raise_()
        self.lineEdit_4.raise_()
        self.pushButton_7.raise_()
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(340, 700, 231, 161))
        self.groupBox.setObjectName("groupBox")
        self.buttonBox = QDialogButtonBox(self.groupBox)
        self.buttonBox.setGeometry(QRect(17, 124, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
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

        self.lineEdit.setText("0.8")
        self.lineEdit_2.setText("0.0")
        self.lineEdit_3.setText("0.9")


        self.label.raise_()
        self.lineEdit.raise_()
        self.label_2.raise_()
        self.lineEdit_2.raise_()
        self.label_3.raise_()
        self.lineEdit_3.raise_()
        self.buttonBox.raise_()
        self.groupBox_4 = QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QRect(10, 10, 1661, 671))
        self.groupBox_4.setObjectName("groupBox_4")

        self.rviz_widget = rvizPython()
        self.horizontalLayout_6 = QHBoxLayout()
        self.groupBox_4.setLayout(self.horizontalLayout_6)
        self.horizontalLayout_6.addWidget(self.rviz_widget)

        self.groupBox_2.raise_()
        self.groupBox_2.raise_()
        self.groupBox_3.raise_()
        self.groupBox.raise_()
        self.groupBox_4.raise_()
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1680, 25))
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

    def on_visualize_prediction_click(self):

        # visualize trajectory
        try:
            rospy.wait_for_service('visualize_trajectory', timeout=2.0)

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
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)
    
    def on_get_context_click(self):
        try:
            rospy.wait_for_service('get_context', timeout=2.0)

            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context
            self.lineEdit_6.setText(str(round(self.context.x, 2)))
            self.lineEdit_5.setText(str(round(self.context.y, 2)))
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
                self.prediction = resp.prediction.poses



            except AttributeError:
                rospy.loginfo("Context not yet extracted!")
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    # old/depricated
    def generalize(self):
        rospy.loginfo("CHECK")
        # get context 
        
        try:
            rospy.wait_for_service('get_context', timeout=2.0)
            get_context = rospy.ServiceProxy('get_context', GetContext)
            resp = get_context()
            self.context = resp.context
            print(self.context)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

        # make prediction using this context
        rospy.wait_for_service('make_prediction', timeout=2.0)

        try:
            make_prediction = rospy.ServiceProxy('make_prediction', MakePrediction)
            resp = make_prediction(self.context)
            self.prediction = resp.prediction.poses

        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)

        # visualize trajectory
        rospy.wait_for_service('visualize_trajectory', timeout=2.0)
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

        self.buttonBox.accepted.connect(self.on_set_object_position_click)

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
        self.pushButton_5.clicked.connect(self.on_predict_click)

        self.pushButton_7.setText(_translate("MainWindow", "Get context"))
        self.pushButton_7.clicked.connect(self.on_get_context_click)

        self.label_4.setText(_translate("MainWindow", "y: "))
        self.label_5.setText(_translate("MainWindow", "x: "))
        self.label_6.setText(_translate("MainWindow", "z: "))
        self.pushButton_8.setText(_translate("MainWindow", "Visualize prediction"))
        self.pushButton_8.clicked.connect(self.on_visualize_prediction_click)

        self.pushButton_9.setText(_translate("MainWindow", "Clear visualizations"))
        self.pushButton_9.clicked.connect(self.on_clear_trajectories_click)

        self.menuOnline_teaching_GUI.setTitle(_translate("MainWindow", "Online teaching GUI"))

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = experimentGUI()
    
    sys.exit(app.exec_())
