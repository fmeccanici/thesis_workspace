
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import rospy, sys

from std_msgs.msg import Bool, Byte
from experiment.msg import FormGUIinteraction
from data_logger.srv import CreateParticipant, ToCsv

class FormGUI(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(FormGUI, self).__init__(*args, **kwargs)
        self._form_gui_interaction_pub = rospy.Publisher('form_gui_interaction', FormGUIinteraction, queue_size=10)

        
        
        self.initUI()
        self.show()

    def initUI(self):
        self.setObjectName("MainWindow")
        self.resize(275, 401)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.radioButton = QRadioButton(self.centralwidget)
        self.radioButton.setGeometry(QRect(90, 60, 117, 22))
        self.radioButton.setObjectName("radioButton")
        self.lineEdit = QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QRect(90, 30, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setGeometry(QRect(20, 60, 67, 17))
        self.label_2.setObjectName("label_2")
        self.frame = QFrame(self.centralwidget)
        self.frame.setGeometry(QRect(10, 10, 261, 341))
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setObjectName("frame")
        self.label_4 = QLabel(self.frame)
        self.label_4.setGeometry(QRect(10, 120, 181, 17))
        self.label_4.setObjectName("label_4")
        self.groupBox = QGroupBox(self.frame)
        self.groupBox.setGeometry(QRect(10, 130, 311, 41))
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.radioButton_3 = QRadioButton(self.groupBox)
        self.radioButton_3.setGeometry(QRect(0, 10, 51, 22))
        self.radioButton_3.setObjectName("radioButton_3")
        self.radioButton_4 = QRadioButton(self.groupBox)
        self.radioButton_4.setGeometry(QRect(50, 10, 51, 22))
        self.radioButton_4.setObjectName("radioButton_4")
        self.radioButton_5 = QRadioButton(self.groupBox)
        self.radioButton_5.setGeometry(QRect(100, 10, 51, 22))
        self.radioButton_5.setObjectName("radioButton_5")
        self.radioButton_6 = QRadioButton(self.groupBox)
        self.radioButton_6.setGeometry(QRect(150, 10, 51, 22))
        self.radioButton_6.setObjectName("radioButton_6")
        self.radioButton_7 = QRadioButton(self.groupBox)
        self.radioButton_7.setGeometry(QRect(200, 10, 51, 22))
        self.radioButton_7.setObjectName("radioButton_7")
        self.label_5 = QLabel(self.frame)
        self.label_5.setGeometry(QRect(10, 240, 181, 17))
        self.label_5.setObjectName("label_5")
        self.groupBox_2 = QGroupBox(self.frame)
        self.groupBox_2.setGeometry(QRect(10, 240, 131, 51))
        self.groupBox_2.setTitle("")
        self.groupBox_2.setObjectName("groupBox_2")
        self.radioButton_13 = QRadioButton(self.groupBox_2)
        self.radioButton_13.setGeometry(QRect(0, 20, 61, 22))
        self.radioButton_13.setObjectName("radioButton_13")
        self.radioButton_14 = QRadioButton(self.groupBox_2)
        self.radioButton_14.setGeometry(QRect(60, 20, 71, 22))
        self.radioButton_14.setObjectName("radioButton_14")
        self.label_6 = QLabel(self.frame)
        self.label_6.setGeometry(QRect(10, 180, 181, 17))
        self.label_6.setObjectName("label_6")
        self.groupBox_3 = QGroupBox(self.frame)
        self.groupBox_3.setGeometry(QRect(10, 190, 311, 41))
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.radioButton_15 = QRadioButton(self.groupBox_3)
        self.radioButton_15.setGeometry(QRect(0, 10, 51, 22))
        self.radioButton_15.setObjectName("radioButton_15")
        self.radioButton_16 = QRadioButton(self.groupBox_3)
        self.radioButton_16.setGeometry(QRect(50, 10, 51, 22))
        self.radioButton_16.setObjectName("radioButton_16")
        self.radioButton_17 = QRadioButton(self.groupBox_3)
        self.radioButton_17.setGeometry(QRect(100, 10, 51, 22))
        self.radioButton_17.setObjectName("radioButton_17")
        self.radioButton_18 = QRadioButton(self.groupBox_3)
        self.radioButton_18.setGeometry(QRect(150, 10, 51, 22))
        self.radioButton_18.setObjectName("radioButton_18")
        self.radioButton_19 = QRadioButton(self.groupBox_3)
        self.radioButton_19.setGeometry(QRect(200, 10, 51, 22))
        self.radioButton_19.setObjectName("radioButton_19")
        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setGeometry(QRect(20, 30, 67, 17))
        self.label_3.setObjectName("label_3")
        self.radioButton_2 = QRadioButton(self.centralwidget)
        self.radioButton_2.setGeometry(QRect(170, 60, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.lineEdit_3 = QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QRect(90, 90, 113, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.buttonBox = QDialogButtonBox(self.centralwidget)
        self.buttonBox.setGeometry(QRect(20, 310, 176, 27))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.label = QLabel(self.centralwidget)
        self.label.setGeometry(QRect(20, 90, 67, 17))
        self.label.setObjectName("label")
        self.frame.raise_()
        self.radioButton.raise_()
        self.lineEdit.raise_()
        self.label_2.raise_()
        self.label_3.raise_()
        self.radioButton_2.raise_()
        self.lineEdit_3.raise_()
        self.buttonBox.raise_()
        self.label.raise_()
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 275, 25))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi(self)
        QMetaObject.connectSlotsByName(self)

    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.radioButton.setText(_translate("MainWindow", "Ma&le"))
        self.label_2.setText(_translate("MainWindow", "Gender"))
        self.label_4.setText(_translate("MainWindow", "Teleoperation experience"))
        self.radioButton_3.setText(_translate("MainWindow", "1"))
        self.radioButton_4.setText(_translate("MainWindow", "2"))
        self.radioButton_5.setText(_translate("MainWindow", "3"))
        self.radioButton_6.setText(_translate("MainWindow", "4"))
        self.radioButton_7.setText(_translate("MainWindow", "5"))
        self.label_5.setText(_translate("MainWindow", "Left/right handed"))
        self.radioButton_13.setText(_translate("MainWindow", "Left"))
        self.radioButton_14.setText(_translate("MainWindow", "Right"))
        self.label_6.setText(_translate("MainWindow", "Keyboard experience"))
        self.radioButton_15.setText(_translate("MainWindow", "1"))
        self.radioButton_16.setText(_translate("MainWindow", "2"))
        self.radioButton_17.setText(_translate("MainWindow", "3"))
        self.radioButton_18.setText(_translate("MainWindow", "4"))
        self.radioButton_19.setText(_translate("MainWindow", "5"))
        self.label_3.setText(_translate("MainWindow", "Number"))
        self.radioButton_2.setText(_translate("MainWindow", "Female"))
        self.label.setText(_translate("MainWindow", "Age"))

        self.buttonBox.accepted.connect(self.onOkclick)
    
    def saveData(self):
        try: 
            rospy.wait_for_service('to_csv')
            to_csv = rospy.ServiceProxy('to_csv', ToCsv)
            number_msg = Byte(int(self.lineEdit.text()))

            resp = to_csv(number_msg)     

        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def createParticipant(self):
        try:
            rospy.wait_for_service('create_participant', timeout=2.0)
            create_participant = rospy.ServiceProxy('create_participant', CreateParticipant)
            
            number = int(self.lineEdit.text())

            number_msg = Byte(number)

            if self.radioButton.isChecked():
                gender = 1
            else:
                gender = 0
            
            gender_msg = Bool(gender)
            age = int(self.lineEdit_3.text())
            age_msg = Byte(age)

            if self.radioButton_3.isChecked():
                teleop_experience = 1
            elif self.radioButton_4.isChecked():
                teleop_experience = 2
            elif self.radioButton_5.isChecked():
                teleop_experience = 3
            elif self.radioButton_6.isChecked():
                teleop_experience = 4
            elif self.radioButton_7.isChecked():
                teleop_experience = 5

            teleop_experience_msg = Byte(teleop_experience)

            if self.radioButton_15.isChecked():
                keyboard_experience = 1
            elif self.radioButton_16.isChecked():
                keyboard_experience = 2
            elif self.radioButton_17.isChecked():
                keyboard_experience = 3
            elif self.radioButton_18.isChecked():
                keyboard_experience = 4
            elif self.radioButton_19.isChecked():
                keyboard_experience = 5

            keyboard_experience_msg = Byte(keyboard_experience)

            if self.radioButton_13.isChecked():
                left_right_handed = 0
            elif self.radioButton_14.isChecked():
                left_right_handed = 1

            left_right_handed_msg = Bool(left_right_handed)

            resp = create_participant(number_msg, gender_msg, age_msg, teleop_experience_msg, keyboard_experience_msg, left_right_handed_msg)     
        
        except (rospy.ServiceException, rospy.ROSException) as e:
            print("Service call failed: %s" %e)

    def onOkclick(self):
        self.createParticipant()
        self.saveData()
if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = FormGUI()
    
    sys.exit(app.exec_())
