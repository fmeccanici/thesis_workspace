
# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import sys
import matplotlib.pyplot as plt
import random


from promp_demo_2d_python.promp_demo_2d import PrompDemo2D 
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

class Demo2dGUI(QMainWindow):
    def __init__(self, *args, **kwargs):

        # inheretance on QMainWindow class
        super(Demo2dGUI, self).__init__(*args, **kwargs)

        # multithreading
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        # a figure instance to plot on
        self.figure = plt.figure()

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
        self.toolbar = NavigationToolbar(self.canvas, self)

        self.promp_demo_2d = PrompDemo2D()
        self.promp_demo_2d.build_model()

        # Just some button connected to `plot` method
        self.button = QPushButton('Plot')
        self.button.clicked.connect(self.plot)

        # initialize Qt GUI
        self.initGUI()
        self.show()

    def initGUI(self):
        self.setObjectName("MainWindow")
        self.resize(800, 600)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QRect(10, 370, 161, 27))
        self.pushButton.setObjectName("pushButton")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(10, 0, 431, 291))
        self.groupBox.setObjectName("groupBox")
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QRect(10, 340, 161, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QRect(10, 400, 161, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.lineEdit = QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QRect(40, 430, 113, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QRect(40, 460, 113, 27))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.label = QLabel(self.centralwidget)
        self.label.setGeometry(QRect(20, 430, 16, 17))
        self.label.setObjectName("label")
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setGeometry(QRect(20, 460, 16, 17))
        self.label_2.setObjectName("label_2")
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi(self)
        QMetaObject.connectSlotsByName(self)


        # set the layout
        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)
        plot_layout.addWidget(self.button)
        # self.groupBox.setLayout(plot_layout)

    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Add to model"))
        self.groupBox.setTitle(_translate("MainWindow", "Plot"))
        self.pushButton_2.setText(_translate("MainWindow", "Refine"))
        self.pushButton_3.setText(_translate("MainWindow", "Predict"))
        self.lineEdit.setText(_translate("MainWindow", "1"))
        self.lineEdit_2.setText(_translate("MainWindow", "-1"))
        self.label.setText(_translate("MainWindow", "y1"))
        self.label_2.setText(_translate("MainWindow", "y2"))
    
        self.pushButton_2.clicked.connect(lambda: self.use_multithread(self.on_refine_click))
    
    # multithread for executing trajectories
    # needed since otherwise the GUI will freeze
    def use_multithread(self, function):
        worker = Worker(function)
        self.threadpool.start(worker)

    def on_refine_click(self):
        plt.ion()

        self.promp_demo_2d.refine()

    def plot(self):
        ''' plot some random stuff '''
        # random data
        data = [random.random() for i in range(10)]

        # instead of ax.hold(False)
        self.figure.clear()

        # create an axis
        ax = self.figure.add_subplot(111)

        # discards the old graph
        # ax.hold(False) # deprecated, see above

        # plot data
        ax.plot(data, '*-')

        # refresh canvas
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = Demo2dGUI()
    
    sys.exit(app.exec_())
