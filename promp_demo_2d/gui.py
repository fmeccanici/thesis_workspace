
# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import sys, time
import matplotlib.pyplot as plt
import random
import numpy as np

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
        
        ####### Set up plot #########
        self.min_x = 0
        self.max_x = 10
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()

        # a figure instance to plot on
        # self.figure = plt.figure()
        
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
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(320, 10, 471, 541))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(0, 10, 221, 271))
        self.groupBox_2.setObjectName("groupBox_2")
        self.lineEdit_3 = QLineEdit(self.groupBox_2)
        self.lineEdit_3.setGeometry(QRect(30, 120, 113, 27))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.label_3 = QLabel(self.groupBox_2)
        self.label_3.setGeometry(QRect(10, 90, 16, 17))
        self.label_3.setObjectName("label_3")
        self.radioButton_6 = QRadioButton(self.groupBox_2)
        self.radioButton_6.setGeometry(QRect(10, 160, 117, 22))
        self.radioButton_6.setObjectName("radioButton_6")
        self.label_4 = QLabel(self.groupBox_2)
        self.label_4.setGeometry(QRect(10, 120, 16, 17))
        self.label_4.setObjectName("label_4")
        self.lineEdit_4 = QLineEdit(self.groupBox_2)
        self.lineEdit_4.setGeometry(QRect(30, 90, 113, 27))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.radioButton_7 = QRadioButton(self.groupBox_2)
        self.radioButton_7.setGeometry(QRect(10, 180, 117, 22))
        self.radioButton_7.setObjectName("radioButton_7")
        self.pushButton_3 = QPushButton(self.groupBox_2)
        self.pushButton_3.setGeometry(QRect(0, 210, 161, 27))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_5 = QPushButton(self.groupBox_2)
        self.pushButton_5.setGeometry(QRect(0, 20, 161, 27))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_4 = QPushButton(self.groupBox_2)
        self.pushButton_4.setGeometry(QRect(0, 50, 161, 27))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_6 = QPushButton(self.groupBox_2)
        self.pushButton_6.setGeometry(QRect(0, 240, 161, 27))
        self.pushButton_6.setObjectName("pushButton_6")
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(10, 290, 281, 361))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton_2 = QPushButton(self.groupBox_3)
        self.pushButton_2.setGeometry(QRect(20, 40, 161, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.radioButton_2 = QRadioButton(self.groupBox_3)
        self.radioButton_2.setGeometry(QRect(40, 230, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.pushButton = QPushButton(self.groupBox_3)
        self.pushButton.setGeometry(QRect(20, 120, 161, 27))
        self.pushButton.setObjectName("pushButton")
        self.radioButton_3 = QRadioButton(self.groupBox_3)
        self.radioButton_3.setGeometry(QRect(40, 180, 231, 22))
        self.radioButton_3.setObjectName("radioButton_3")
        self.radioButton_4 = QRadioButton(self.groupBox_3)
        self.radioButton_4.setGeometry(QRect(40, 70, 117, 22))
        self.radioButton_4.setObjectName("radioButton_4")
        self.label_5 = QLabel(self.groupBox_3)
        self.label_5.setGeometry(QRect(25, 200, 41, 20))
        self.label_5.setObjectName("label_5")
        self.radioButton_5 = QRadioButton(self.groupBox_3)
        self.radioButton_5.setGeometry(QRect(40, 90, 117, 22))
        self.radioButton_5.setObjectName("radioButton_5")
        self.lineEdit_5 = QLineEdit(self.groupBox_3)
        self.lineEdit_5.setGeometry(QRect(70, 200, 113, 27))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.radioButton = QRadioButton(self.groupBox_3)
        self.radioButton.setGeometry(QRect(40, 150, 231, 22))
        self.radioButton.setObjectName("radioButton")
        self.radioButton_2.raise_()
        self.pushButton_2.raise_()
        self.pushButton.raise_()
        self.radioButton_3.raise_()
        self.radioButton_4.raise_()
        self.radioButton_5.raise_()
        self.label_5.raise_()
        self.lineEdit_5.raise_()
        self.radioButton.raise_()
        self.pushButton_2.raise_()
        self.radioButton_2.raise_()
        self.pushButton.raise_()
        self.radioButton_3.raise_()
        self.radioButton_4.raise_()
        self.radioButton_5.raise_()
        self.label_5.raise_()
        self.lineEdit_5.raise_()
        self.radioButton.raise_()
        self.pushButton_2.raise_()
        self.radioButton_2.raise_()
        self.pushButton.raise_()
        self.radioButton_3.raise_()
        self.radioButton_4.raise_()
        self.label_5.raise_()
        self.radioButton_5.raise_()
        self.lineEdit_5.raise_()
        self.radioButton.raise_()
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
        self.groupBox.setLayout(plot_layout)

    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        self.groupBox.setTitle(_translate("MainWindow", "Plot"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Learning from Demonstration"))
        self.lineEdit_3.setText(_translate("MainWindow", "-1"))
        self.label_3.setText(_translate("MainWindow", "y1"))
        self.radioButton_6.setText(_translate("MainWindow", "No obstacle"))
        self.label_4.setText(_translate("MainWindow", "y2"))
        self.lineEdit_4.setText(_translate("MainWindow", "1"))
        self.radioButton_7.setText(_translate("MainWindow", "Obstacle"))
        self.pushButton_3.setText(_translate("MainWindow", "Predict"))
        self.pushButton_5.setText(_translate("MainWindow", "Set context"))
        self.pushButton_4.setText(_translate("MainWindow", "Demonstrate"))
        self.pushButton_6.setText(_translate("MainWindow", "Enable keyboard"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Refinement"))
        self.pushButton_2.setText(_translate("MainWindow", "Refine"))
        self.radioButton_2.setText(_translate("MainWindow", "Normal"))
        self.pushButton.setText(_translate("MainWindow", "Add to model"))
        self.radioButton_3.setText(_translate("MainWindow", "Welford: forgetting factor"))
        self.radioButton_4.setText(_translate("MainWindow", "No obstacle"))
        self.label_5.setText(_translate("MainWindow", "alpha"))
        self.radioButton_5.setText(_translate("MainWindow", "Obstacle"))
        self.lineEdit_5.setText(_translate("MainWindow", "1"))
        self.radioButton.setText(_translate("MainWindow", "Welford: no forgetting factor"))

        self.pushButton_2.clicked.connect(lambda: self.use_multithread(self.on_refine_click))
        self.pushButton_4.clicked.connect(self.on_demonstrate_click)
        self.pushButton_5.clicked.connect(lambda: self.use_multithread(self.on_set_context_click))
        self.pushButton_6.clicked.connect(lambda: self.use_multithread(self.on_enable_keyboard_click))

    # multithread for executing trajectories
    # needed since otherwise the GUI will freeze
    def use_multithread(self, function):
        worker = Worker(function)
        self.threadpool.start(worker)
    
    def on_set_context_click(self):
        y1 = float(self.lineEdit_4.text())
        y2 = float(self.lineEdit_3.text())

        self.context = [y1, y2]
    
    def on_running(self, xdata, ydata):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def on_demonstrate_click(self):
        self.promp_demo_2d.demonstrate(self.context, self.figure, self.canvas)

    def on_refine_click(self):
        self.promp_demo_2d.refine()

    def on_enable_keyboard_click(self):
        self.promp_demo_2d.enable_keyboard()

    def plot(self):
        ''' plot some random stuff '''
        plt.ion()

        # random data
        data = [random.random() for i in range(10)]
        xdata = []
        ydata = []

        for x in np.arange(0,10,0.5):
            # data[j] += j
            # instead of ax.hold(False)
            # self.figure.clear()

            # create an axis
            # ax = self.figure.add_subplot(111)

            # discards the old graph
            # ax.hold(False) # deprecated, see above

            # plot data
            # ax.plot(data, '*-')

            # refresh canvas
            # self.canvas.draw()
            xdata.append(x)
            ydata.append(np.exp(-x**2)+10*np.exp(-(x-7)**2))
            self.on_running(xdata, ydata)
            time.sleep(1)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = Demo2dGUI()
    
    sys.exit(app.exec_())
