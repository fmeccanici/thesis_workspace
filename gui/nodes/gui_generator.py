# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# other packages
import rospy, sys, roslaunch, rospkg, os

# ROS messages
from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, SetObject, SetObjectResponse
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point

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
        self.setObjectName("MainWindow")
        self.resize(800, 600)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(0, 0, 231, 161))
        self.groupBox.setObjectName("groupBox")
        self.buttonBox = QDialogButtonBox(self.groupBox)
        self.buttonBox.setGeometry(QRect(30, 120, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)

        self.buttonBox.setObjectName("buttonBox")
        self.label = QLabel(self.groupBox)
        self.label.setGeometry(QRect(40, 30, 67, 17))
        self.label.setObjectName("label")
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setGeometry(QRect(40, 60, 67, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QLabel(self.groupBox)
        self.label_3.setGeometry(QRect(40, 90, 67, 17))
        self.label_3.setObjectName("label_3")
        self.lineEdit = QLineEdit(self.groupBox)
        self.lineEdit.setGeometry(QRect(60, 30, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QLineEdit(self.groupBox)
        self.lineEdit_2.setGeometry(QRect(60, 60, 113, 27))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_3 = QLineEdit(self.groupBox)
        self.lineEdit_3.setGeometry(QRect(60, 90, 113, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(230, 0, 181, 141))
        self.groupBox_2.setObjectName("groupBox_2")


        # set buttons
        self.pushButton = QPushButton(self.groupBox_2)
        self.pushButton.setGeometry(QRect(20, 40, 141, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QPushButton(self.groupBox_2)
        self.pushButton_2.setGeometry(QRect(20, 80, 141, 31))
        self.pushButton_2.setObjectName("pushButton_2")

        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(0, 180, 381, 211))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton_3 = QPushButton(self.groupBox_3)
        self.pushButton_3.setGeometry(QRect(20, 40, 141, 31))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QPushButton(self.groupBox_3)
        self.pushButton_4.setGeometry(QRect(20, 80, 141, 31))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QPushButton(self.groupBox_3)
        self.pushButton_5.setGeometry(QRect(20, 120, 141, 31))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QPushButton(self.groupBox_3)
        self.pushButton_6.setGeometry(QRect(20, 160, 141, 31))
        self.pushButton_6.setObjectName("pushButton_6")


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



    def retranslateUi(self):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Object position"))

        self.buttonBox.accepted.connect(self.set_object_position)

        self.label.setText(_translate("MainWindow", "x: "))
        self.label_2.setText(_translate("MainWindow", "y: "))
        self.label_3.setText(_translate("MainWindow", "z: "))

        self.groupBox_2.setTitle(_translate("MainWindow", "Refinement"))
        self.pushButton.setText(_translate("MainWindow", "Start node"))

        self.pushButton_2.setText(_translate("MainWindow", "Stop node"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Learning from Demonstration"))
        self.pushButton_3.setText(_translate("MainWindow", "Start node"))
        self.pushButton_3.clicked.connect(self.start_lfd_node)

        self.pushButton_4.setText(_translate("MainWindow", "Stop node"))
        self.pushButton_4.clicked.connect(self.stop_lfd_node)

        self.pushButton_5.setText(_translate("MainWindow", "Generalize"))
        self.pushButton_6.setText(_translate("MainWindow", "Add demonstration"))

        self.menuOnline_teaching_GUI.setTitle(_translate("MainWindow", "Online teaching GUI"))

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = experimentGUI()
    
    sys.exit(app.exec_())
