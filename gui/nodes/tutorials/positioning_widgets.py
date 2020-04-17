from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import sys

class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.setWindowTitle("Online learning GUI")

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.formGroupBox)
        self.setLayout(mainLayout)


        self.setLayout(layout)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        widget.show()
if __name__ == '__main__':
    app = QApplication(sys.argv)
    # dialog = Dialog()
    # sys.exit(dialog.exec_())
    sys.exit(app.exec_())