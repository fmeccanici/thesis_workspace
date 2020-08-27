import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QWidget, QSlider, QLabel)

class NASATLX(QWidget):
    def __init__(self, parent=None):
        super(NASATLX, self).__init__(parent)
        self.labels = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration']
        grid = QGridLayout()
        
        for i,label in enumerate(self.labels):
            grid.addWidget(self.createSlider(label), i, 0)


        self.setLayout(grid)

        self.setWindowTitle("PyQt5 Sliders")
        self.resize(400, 300)

    def createSlider(self, label):
        groupBox = QGroupBox("")

        label1 = QLabel(label)

        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setMinimum(1)
        slider.setMaximum(21)
        slider.setTickInterval(1)
        
        slider.setSingleStep(1)

        vbox = QVBoxLayout()
        vbox.addWidget(label1)
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)

        return groupBox

if __name__ == '__main__':
    app = QApplication(sys.argv)
    clock = NASATLX()
    clock.show()
    sys.exit(app.exec_())