from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# Only needed for access to command line arguments
import sys

class SetObjectPosition(QWidget):
    def __init__self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.setWindowTitle("My Awesome App")
        self.createFormGroupBox()

        main_layout = QGridLayout()
        main_layout.addWidget(self.formGroupBox, 0, 0)
        main_layout.addWidget(self.formGroupBox, 1, 1)

        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)


        # layout.addWidget(Color('red'), 0, 0)
        # layout.addWidget(Color('green'), 1, 0)
        # layout.addWidget(Color('blue'), 1, 1)
        # layout.addWidget(Color('purple'), 2, 1)

        # widget = QWidget()
        # widget.setLayout(layout)
        # self.setCentralWidget(widget)

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

# You need one (and only one) QApplication instance per application.
# Pass in sys.argv to allow command line arguments for your app.
# If you know you won't use command line arguments QApplication([]) works too.
app = QApplication(sys.argv)

window = MainWindow()
window.show() # IMPORTANT!!!!! Windows are hidden by default.

# Start the event loop.
app.exec_()


# Your application won't reach here until you exit and the event 
# loop has stopped.