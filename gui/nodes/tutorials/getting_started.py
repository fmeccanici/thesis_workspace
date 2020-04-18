## https://techwithtim.net/tutorials/pyqt5-tutorial/basic-gui-application/

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel

import sys

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow,self).__init__()
        self.initUI()

    def initUI(self):

        self.setGeometry(200,200,300,300) # sets the windows x, y, width, height
        self.setWindowTitle("My first window!") # setting the window titl
        
        self.label = QLabel(self)
        self.label.setText("my first label")
        self.label.move(50, 50)  # x, y from top left hand corner.


        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setText("click me")
        self.b1.move(100,100) # to move the button
        self.b1.clicked.connect(self.button_clicked)  # inside main function 


    def button_clicked(self):
        print("clicked") # we will just print clicked when the button is pressed
        self.label.setText("you pressed the button")
        self.update()
        
    def update(self):
        self.label.adjustSize()
def window():
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())

window()