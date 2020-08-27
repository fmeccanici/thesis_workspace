import sys
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QLabel)

from random import randint

class Slider(QSlider):
    minimumChanged = pyqtSignal(int)
    maximumChanged = pyqtSignal(int)

    def setMinimum(self, minimum):
        self.minimumChanged.emit(minimum)
        super(Slider, self).setMinimum(minimum)

    def setMaximum(self, maximum):
        self.maximumChanged.emit(maximum)
        super(Slider, self).setMaximum(maximum)

class NASATLX(QWidget):
    def __init__(self, parent=None):
        super(NASATLX, self).__init__(parent)
        self.labels = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration']

        self.comparison_pairs = [('Effort', 'Performance'), ('Temporal Demand', 'Frustration'), ('Temporal Demand', 'Effort'),
                                ('Physical Demand', 'Frustration'), ('Performance', 'Frustration'), ('Physical Demand', 'Temporal Demand'),
                                ('Physical Demand', 'Performance'), ('Temporal Demand', 'Mental Demand'), ('Frustration', 'Effort'), 
                                ('Performance', 'Mental Demand'), ('Performance', 'Temporal Demand'), ('Mental Demand', 'Effort'), 
                                ('Mental Demand', 'Physical Demand'), ('Effort', 'Physical Demand'), ('Frustration', 'Mental Demand')]

        self.grid = QGridLayout()
        
        self.createRatingSheet()
        self.createComparisonCards()

        self.setLayout(self.grid)

        self.setWindowTitle("PyQt5 Sliders")
        self.resize(400, 300)

    def createRatingSheet(self):
        for i,label in enumerate(self.labels):
            self.grid.addWidget(self.createSlider(label), i, 0)

    def createComparisonCards(self):

        for i, pair in enumerate(self.comparison_pairs):
            groupbox = QGroupBox()

            comparison_radio_hbox = QHBoxLayout()
            radio1 = QRadioButton(pair[0])
            radio2 = QRadioButton(pair[1])
            comparison_radio_hbox.addWidget(radio1)
            comparison_radio_hbox.addWidget(radio2)

            groupbox.setLayout(comparison_radio_hbox)
            
            self.grid.addWidget(groupbox, i, 1)
    
    def createSlider(self, label):
        groupBox = QGroupBox("")
        label1 = QLabel(label)

        self.label = QLabel(alignment=Qt.AlignCenter)

        self.slider = Slider(tickPosition=QSlider.TicksLeft,
            orientation=Qt.Horizontal)
            
        slider_vbox = QVBoxLayout()
        slider_hbox = QHBoxLayout()
        slider_hbox.setContentsMargins(0, 0, 0, 0)
        slider_vbox.setContentsMargins(0, 0, 0, 0)
        slider_vbox.setSpacing(0)

        label_minimum = QLabel(alignment=Qt.AlignLeft)
        self.slider.minimumChanged.connect(label_minimum.setNum)
        label_maximum = QLabel(alignment=Qt.AlignRight)
        self.slider.maximumChanged.connect(label_maximum.setNum)
        slider_vbox.addWidget(self.slider)
        slider_vbox.addLayout(slider_hbox)
        slider_hbox.addWidget(label_minimum, Qt.AlignLeft)
        slider_hbox.addWidget(label_maximum, Qt.AlignRight)
        slider_vbox.addStretch()

        self.slider.setMinimum(1)
        self.slider.setMaximum(21)
        self.slider.setValue(randint(1,21))
        self.slider.setTickInterval(1)
        self.slider.setSingleStep(1)

        vbox = QVBoxLayout(self)
        vbox.addWidget(label1)
        vbox.addLayout(slider_vbox)
        vbox.addWidget(self.label)
        self.setGeometry(300, 300, 300, 150)
        self.slider.valueChanged.connect(self.label.setNum)

        groupBox.setLayout(vbox)

        return groupBox

if __name__ == '__main__':
    app = QApplication(sys.argv)
    clock = NASATLX()
    clock.show()
    sys.exit(app.exec_())