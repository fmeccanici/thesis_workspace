from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog,
QDialogButtonBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
QLabel, QLineEdit, QMenu, QMenuBar, QPushButton, QSpinBox, QTextEdit,
QVBoxLayout, QMessageBox)
from PyQt5.QtCore import pyqtSlot
import sys, rospy

from learning_from_demonstration.srv import AddDemonstration, AddDemonstrationResponse, MakePrediction, MakePredictionResponse, SetObject, SetObjectResponse
from geometry_msgs.msg import PoseStamped, WrenchStamped, PoseArray, Pose, Point


class SetObjectPosition(QDialog):
    NumGridRows = 3
    NumButtons = 4

    def __init__(self):
        super(SetObjectPosition, self).__init__()

        self.createFormGroupBox()

        button_box_object = QDialogButtonBox(QDialogButtonBox.Ok)
        button_box_object.accepted.connect(self.accept_object_pos)
        # buttonBox.rejected.connect(self.reject)
        
        mainLayout.addWidget(self.formGroupBox)
        mainLayout.addWidget(button_box_object)

        self.setLayout(mainLayout)
        
        self.setWindowTitle("Online learning GUI")

    def createFormGroupBox(self):
        
        self.formGroupBox = QGroupBox("Set object position")
        layout = QFormLayout()
        self.x_pos = QLineEdit()
        self.y_pos = QLineEdit()
        self.z_pos = QLineEdit()
        layout.addRow(QLabel("x:"), self.x_pos)
        layout.addRow(QLabel("y:"), self.y_pos)
        layout.addRow(QLabel("z:"), self.z_pos)

        self.formGroupBox.setLayout(layout)
    
    def accept_object_pos(self):
        object_position = Point()
        object_position.x = float(self.x_pos.text())
        object_position.y = float(self.y_pos.text())
        object_position.z = float(self.z_pos.text())

        rospy.wait_for_service('set_object')
        try:
            set_object = rospy.ServiceProxy('set_object', SetObject)
            resp = set_object(object_position)
            return resp.success

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

class GUI():
    def __init__(self):
        rospy.init_node('gui')
        self.app = QApplication(sys.argv)
        self.set_object_position = SetObjectPosition()

        sys.exit(self.set_object_position.exec_())

    def set_layout(self):
        mainLayout = QVBoxLayout()


    def add_widgets(self):

if __name__ == "__main__":
    gui = GUI()





    
