
# import Qt stuff
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import sys, time, ast
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
        self.max_x = 4
        self.min_y = -2
        self.max_y = 2

        self.figure, self.ax = plt.subplots()
        self.lines1, = self.ax.plot([],[], 'r', zorder=0, linewidth=5)
        self.lines2, = self.ax.plot([],[], 'g', zorder=10, linewidth=5)
        
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(self.min_x, self.max_x)
        self.ax.set_ylim(self.min_y, self.max_y)
        self.ax.set_ylabel("y")
        self.ax.set_xlabel("x")

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
        
        self.use_multithread(self.on_enable_keyboard_click)

        self.geometries = []
        self.refined_prediction = []

        # initialize Qt GUI
        self.initGUI()
        self.show()

    def initGUI(self):
        self.setObjectName("MainWindow")
        self.resize(1166, 933)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QRect(370, 10, 791, 781))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QRect(0, 10, 451, 271))
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
        self.pushButton_10 = QPushButton(self.groupBox_2)
        self.pushButton_10.setGeometry(QRect(170, 20, 161, 27))
        self.pushButton_10.setObjectName("pushButton_10")
        self.pushButton_11 = QPushButton(self.groupBox_2)
        self.pushButton_11.setGeometry(QRect(170, 50, 161, 27))
        self.pushButton_11.setObjectName("pushButton_11")
        self.groupBox_3 = QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QRect(10, 290, 361, 361))
        self.groupBox_3.setObjectName("groupBox_3")
        self.pushButton_2 = QPushButton(self.groupBox_3)
        self.pushButton_2.setGeometry(QRect(20, 90, 161, 27))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton = QPushButton(self.groupBox_3)
        self.pushButton.setGeometry(QRect(20, 120, 161, 27))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_7 = QPushButton(self.groupBox_3)
        self.pushButton_7.setGeometry(QRect(190, 30, 161, 27))
        self.pushButton_7.setObjectName("pushButton_7")
        self.lineEdit = QLineEdit(self.groupBox_3)
        self.lineEdit.setGeometry(QRect(190, 60, 161, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton_8 = QPushButton(self.groupBox_3)
        self.pushButton_8.setGeometry(QRect(190, 120, 161, 27))
        self.pushButton_8.setObjectName("pushButton_8")
        self.groupBox_4 = QGroupBox(self.groupBox_3)
        self.groupBox_4.setGeometry(QRect(30, 150, 231, 121))
        self.groupBox_4.setObjectName("groupBox_4")
        self.radioButton_2 = QRadioButton(self.groupBox_4)
        self.radioButton_2.setGeometry(QRect(0, 100, 117, 22))
        self.radioButton_2.setObjectName("radioButton_2")
        self.radioButton_3 = QRadioButton(self.groupBox_4)
        self.radioButton_3.setGeometry(QRect(0, 50, 231, 22))
        self.radioButton_3.setObjectName("radioButton_3")
        self.label_5 = QLabel(self.groupBox_4)
        self.label_5.setGeometry(QRect(0, 70, 41, 20))
        self.label_5.setObjectName("label_5")
        self.lineEdit_5 = QLineEdit(self.groupBox_4)
        self.lineEdit_5.setGeometry(QRect(45, 70, 113, 27))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.radioButton = QRadioButton(self.groupBox_4)
        self.radioButton.setGeometry(QRect(0, 20, 231, 22))
        self.radioButton.setObjectName("radioButton")
        self.pushButton_9 = QPushButton(self.groupBox_3)
        self.pushButton_9.setGeometry(QRect(190, 90, 161, 27))
        self.pushButton_9.setObjectName("pushButton_9")
        self.pushButton_12 = QPushButton(self.groupBox_3)
        self.pushButton_12.setGeometry(QRect(20, 280, 161, 27))
        self.pushButton_12.setObjectName("pushButton_12")
        self.pushButton_13 = QPushButton(self.groupBox_3)
        self.pushButton_13.setGeometry(QRect(20, 30, 161, 27))
        self.pushButton_13.setObjectName("pushButton_13")
        self.lineEdit_2 = QLineEdit(self.groupBox_3)
        self.lineEdit_2.setGeometry(QRect(20, 60, 161, 27))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.pushButton.raise_()
        self.pushButton_2.raise_()
        self.pushButton_7.raise_()
        self.lineEdit.raise_()
        self.pushButton_8.raise_()
        self.groupBox_4.raise_()
        self.pushButton_9.raise_()
        self.pushButton_12.raise_()
        self.pushButton_13.raise_()
        self.lineEdit_2.raise_()
        self.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(self)
        self.menubar.setGeometry(QRect(0, 0, 1198, 25))
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
        self.pushButton_10.setText(_translate("MainWindow", "Random context"))
        self.pushButton_11.setText(_translate("MainWindow", "Initialize model"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Refinement"))
        self.pushButton_2.setText(_translate("MainWindow", "Refine"))
        self.pushButton.setText(_translate("MainWindow", "Add to model"))
        self.pushButton_7.setText(_translate("MainWindow", "Load trajectory"))
        self.lineEdit.setText(_translate("MainWindow", "demonstration_1"))
        self.pushButton_8.setText(_translate("MainWindow", "Plot refinement"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Model adding method"))
        self.radioButton_2.setText(_translate("MainWindow", "Normal"))
        self.radioButton_3.setText(_translate("MainWindow", "Welford: forgetting factor"))
        self.label_5.setText(_translate("MainWindow", "alpha"))
        self.lineEdit_5.setText(_translate("MainWindow", "1"))
        self.radioButton.setText(_translate("MainWindow", "Welford: no forgetting factor"))
        self.pushButton_9.setText(_translate("MainWindow", "Clear plots"))
        self.pushButton_12.setText(_translate("MainWindow", "Done refining"))
        self.pushButton_13.setText(_translate("MainWindow", "Save trajectory"))
        self.lineEdit_2.setText(_translate("MainWindow", "demonstration_1"))

        self.pushButton_2.clicked.connect(lambda: self.use_multithread(self.on_refine_click))
        self.pushButton_4.clicked.connect(self.on_demonstrate_click)
        self.pushButton_5.clicked.connect(lambda: self.use_multithread(self.on_set_context_click))
        self.pushButton_6.clicked.connect(lambda: self.use_multithread(self.on_enable_keyboard_click))
        self.pushButton_3.clicked.connect(self.on_predict_click)
        self.pushButton.clicked.connect(self.on_add_to_model_click)
        self.pushButton_8.clicked.connect(self.on_plot_refinement_click)
        self.pushButton_9.clicked.connect(self.on_clear_plots_click)
        self.pushButton_7.clicked.connect(self.on_load_trajectory_click)
        self.pushButton_10.clicked.connect(self.on_set_random_context_click)
        self.lineEdit.setText(_translate("MainWindow", "/good/demonstration_2"))
        self.pushButton_11.clicked.connect(self.on_initialize_model_click)
        self.pushButton_12.clicked.connect(self.on_done_refining_click)

        # self.radioButton_6.setChecked(1)
        # self.radioButton_4.setChecked(1)
        self.radioButton.setChecked(1)

    # multithread for executing trajectories
    # needed since otherwise the GUI will freeze
    def use_multithread(self, function):
        worker = Worker(function)
        self.threadpool.start(worker)
    
    def on_done_refining_click(self):
        # set refined prediction to empty --> needed to reset the refinement plotting
        self.refined_prediction = []
    
    def on_set_context_click(self):
        y1 = float(self.lineEdit_4.text())
        y2 = float(self.lineEdit_3.text())

        self.context = [y1, y2]
    def on_set_random_context_click(self):

        y1 = random.randrange(self.min_y+1, self.max_y-1, 1)
        y2 = random.randrange(self.min_y+1, self.max_y-1, 1)


        self.context = [y1, y2]

        self.lineEdit_4.setText(str(y1))
        self.lineEdit_3.setText(str(y2))

    def add_geometries(self):
        for geometry in self.geometries:
            geometry.remove()
        self.geometries = []
        
        t_plot = self.promp_demo_2d.t[20]
        y_plot = -1.0
        context1 = [2.0, self.context[0]]
        context2 = [3.6, self.context[1]]
        
        diameter = 0.1

        circle1 = plt.Circle((context1[0], context1[1]), diameter, color='b', fill=False, linewidth=2)
        circle2 = plt.Circle((context2[0], context2[1]), diameter, color='b', fill=False, linewidth=2)   

        self.geometries.append(circle1)
        self.geometries.append(circle2)

        if self.radioButton_7.isChecked():
            obstacle = plt.Rectangle((t_plot, y_plot), 0.5, 2, linewidth=1, fill=True)
            self.geometries.append(obstacle)
        
        elif self.radioButton_6.isChecked():
            pass
        
        for geometry in self.geometries:
            self.ax.add_artist(geometry)
            print("Added " + str(geometry))
    
    def clear_plot(self):
        for geometry in self.geometries:
            geometry.remove()

    def on_running(self, xdata, ydata, geometries, which_line=1):
        if which_line == 1:

            #Update data (with the new _and_ the old points)
            self.lines1.set_xdata(xdata)
            self.lines1.set_ydata(ydata)
        elif which_line == 2:
            self.lines2.set_xdata(xdata)
            self.lines2.set_ydata(ydata)

        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()


        t = time.time()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        elapsed = time.time() - t

        return elapsed
    def on_initialize_model_click(self):
        self.promp_demo_2d.build_model()
        
    def on_load_trajectory_click(self):
        path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/promp_demo_2d/data/'
        file_name = str(self.lineEdit.text()) + '.txt'
        with open(path+file_name, "r") as demo:
            read_demo = ast.literal_eval(demo.read())

            self.refined_prediction = read_demo[0]
            self.context = read_demo[1]
        
        y1 = self.lineEdit_4.setText(str(int(self.context[0])))
        y2 = self.lineEdit_3.setText(str(int(self.context[1])))
        
    def on_demonstrate_click(self):
        self.promp_demo_2d.mode = 1

        self.on_set_context_click()
        self.add_geometries()
        for i in range(len(self.promp_demo_2d.t)):
            try:    
                self.promp_demo_2d.ode_update_step(i)
                xdata = self.promp_demo_2d.t[:i]
                ydata = self.promp_demo_2d.y[:i]
                
                self.on_running(xdata, ydata, self.geometries, 1)
                time.sleep(self.promp_demo_2d.dt)
            except IndexError:
                plt.close()

        self.promp_demo_2d.demonstrate(self.context, self.figure, self.canvas)

    def on_predict_click(self):


        self.on_set_context_click()
        self.prediction = np.asarray(self.promp_demo_2d.generalize(self.context))
        
        xdata = self.promp_demo_2d.t
        ydata = self.prediction

        self.add_geometries()
        self.on_running(xdata, ydata, self.geometries, 1)



    def on_clear_plots_click(self):

        for geometry in self.geometries:
            print("Cleared " + str(geometry))
            geometry.remove()
        
        self.geometries = []
        self.lines1.remove()
        self.lines2.remove()

        self.lines1, = self.ax.plot([],[], 'r', zorder=0, linewidth=5)
        self.lines2, = self.ax.plot([],[], 'g', zorder=10, linewidth=5)

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def on_refine_click(self):
        self.promp_demo_2d.mode = 0
        if self.refined_prediction == []:
            print("No refinement done yet")
            self.promp_demo_2d.y = self.prediction   
        else: 
            print("Refine refinement")
            self.promp_demo_2d.y = self.refined_prediction   
        
        self.on_clear_plots_click()

        xdata = self.promp_demo_2d.t
        ydata = self.promp_demo_2d.y

        self.add_geometries()
        self.on_running(xdata, ydata, self.geometries, 1)

        t0 = time.time()
        elapsed = 0
        for i in range(len(self.promp_demo_2d.t)-1):
            try:    

                self.promp_demo_2d.refine_update_step(i)

                xdata = self.promp_demo_2d.t[:i]
                ydata = self.promp_demo_2d.y[:i]
                # print("y = " + str(ydata[-1]))
                elapsed += self.on_running(xdata, ydata, self.geometries, 2)

                time.sleep(self.promp_demo_2d.dt)
            except IndexError:
                pass
        
        print("Elapsed time = " + str(elapsed))
        refined_prediction = self.promp_demo_2d.y
        alpha = 1
        elapsed = time.time() - t0
        print("Total elapsed time = " + str(elapsed))

        if self.refined_prediction == []:
            print("No refinement done yet")
            self.refined_prediction = np.add(np.asarray(self.prediction), alpha * np.subtract(np.asarray(self.prediction), refined_prediction))
        else:
            print("Refine refinement")
            self.refined_prediction = np.add(np.asarray(self.refined_prediction), alpha * np.subtract(np.asarray(self.refined_prediction), refined_prediction))
            

    def on_plot_refinement_click(self):
        # self.on_clear_plots_click()

        self.lines2.set_xdata(self.promp_demo_2d.t)
        self.lines2.set_ydata(self.refined_prediction)

        # self.ax.plot(self.promp_demo_2d.t, self.refined_prediction, 'go')
        self.figure.canvas.draw()

        print("Refinement plotted")

    def on_add_to_model_click(self):
        demonstration = ( list(self.refined_prediction), self.context )
        alpha = float(self.lineEdit_5.text())
        if self.radioButton.isChecked():
            self.promp_demo_2d.promp.welford_update((np.asarray([demonstration[0]]).T, demonstration[1] ))
        elif self.radioButton_2.isChecked():
            self.promp_demo_2d.promp.add_demonstration((np.asarray([demonstration[0]]).T, demonstration[1] ))
        elif self.radioButton_3.isChecked():
            self.promp_demo_2d.promp.welford_update((np.asarray([demonstration[0]]).T, demonstration[1] ), alpha=alpha)

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
