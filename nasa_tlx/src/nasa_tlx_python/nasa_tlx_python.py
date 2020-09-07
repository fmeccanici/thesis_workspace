import sys, os
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QMenu, QPushButton, QRadioButton, QVBoxLayout, QHBoxLayout, QWidget, QSlider, QLabel,
                             QLineEdit)

from random import randint, choice

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
        self.base_path = '/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/data/'

        self.methods_labels = ['online+omni', 'offline+omni', 'online+pendant', 'offline+pendant']
        self.methods_radio_buttons = []

        self.labels = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration']
        self.rating_labels_with_description = ['Mental Demand                How mentally demanding was the task?',
                                               'Physical Demand              How physically demanding was the task?',
                                               'Temporal Demand              How hurried or rushed was the pace of the task?',
                                               'Performance                  How successful were you in accomplishing what you were asked to do?',
                                               'Effort                       How hard did you have to work to accomplish your level of performance?',
                                               'Frustration                  How insecure, discouraged, irritated, stressed and annoyed were you?']

        self.comparison_pairs = [('Effort', 'Performance'), ('Temporal Demand', 'Frustration'), ('Temporal Demand', 'Effort'),
                                ('Physical Demand', 'Frustration'), ('Performance', 'Frustration'), ('Physical Demand', 'Temporal Demand'),
                                ('Physical Demand', 'Performance'), ('Temporal Demand', 'Mental Demand'), ('Frustration', 'Effort'), 
                                ('Performance', 'Mental Demand'), ('Performance', 'Temporal Demand'), ('Mental Demand', 'Effort'), 
                                ('Mental Demand', 'Physical Demand'), ('Effort', 'Physical Demand'), ('Frustration', 'Mental Demand')]

        self.comparison_pairs_radio_buttons = []
        self.rating_sliders = []

        self.data = {'Ratings': {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Comparisons": {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Workload": 0}


        self.grid_rating_sheet = QGridLayout()
        self.grid_comparison_cards = QGridLayout()

        self.main_layout = QHBoxLayout()

        self.createRatingSheet()
        self.createComparisonCards()
        self.createDataStoreSheet()

        self.main_layout.addLayout(self.grid_rating_sheet)
        self.main_layout.addLayout(self.grid_comparison_cards)

        self.setLayout(self.main_layout)

        self.reset_button.clicked.connect(self.onResetClick)
        self.store_button.clicked.connect(self.onStoreClick)

        self.setRandomValues()

        self.setWindowTitle("NASA TLX")
        self.resize(400, 300)

    def setDataPath(self):
        participant_number = int(self.participant_number_text.text())
        
        for i, radio_button in enumerate(self.methods_radio_buttons):
            if radio_button.isChecked():
                method = self.methods_labels[i]

        path = self.base_path + 'participant_' + str(participant_number) + '/nasa_tlx/' + str(method) + '/'
        
        if not os.path.isdir(path):
            os.makedirs(path)

        self.data_path = path

    def storeData(self):
        with open(self.data_path + 'data.txt', 'w+') as f:
            f.write(str(self.data))
    
    def setRandomValues(self):
        for slider in self.rating_sliders:
            slider.setValue(choice([0, 20, 40, 60, 80, 100]))

        for radio_button_pair in self.comparison_pairs_radio_buttons:
            position = randint(0,1)
            radio_button_pair[position].setChecked(True)
        
        self.data = {'Ratings': {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Comparisons": {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Workload": 0}

    def onResetClick(self):
        self.setRandomValues()

    def isValidScore(self):
        total_count = 0
        for label in self.labels:
            if self.data["Comparisons"][label] > 5:
                return False
            total_count += self.data["Comparisons"][label]

        if total_count == 15:
            return True
        else:
            print("Count = " + str(total_count))
            return False

    def calculateWorkload(self):
        sum_adjusted_ratings = 0
        for label in self.labels:
            sum_adjusted_ratings += self.data["Ratings"][label] * self.data["Comparisons"][label] 

        return sum_adjusted_ratings / 15

    def onStoreClick(self):
        try:
            self.setDataPath()
            for i, pair in enumerate(self.comparison_pairs):
                if self.comparison_pairs_radio_buttons[i][0].isChecked():
                    self.data["Comparisons"][self.comparison_pairs[i][0]] += 1
                elif self.comparison_pairs_radio_buttons[i][1].isChecked():
                    self.data["Comparisons"][self.comparison_pairs[i][1]] += 1

            for i, label in enumerate(self.labels):
                self.data["Ratings"][label] = self.rating_sliders[i].value()

            if not self.isValidScore():
                print("No valid sources of workload score!")
            else:
                workload = self.calculateWorkload()
                print("Workload = " + str(workload))
                self.data["Workload"] = workload

                self.storeData()

                self.data = {'Ratings': {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Comparisons": {"Effort": 0, "Performance": 0, "Temporal Demand": 0, "Frustration": 0, "Physical Demand": 0, "Mental Demand": 0},
                    "Workload": 0}

        except Exception as e:
            print("Storing failure: " + str(e))

    def createDataStoreSheet(self):
        groupBox = QGroupBox("Store data")
        grid = QGridLayout()

        self.participant_number_text = QLineEdit()
        self.store_button = QPushButton("Store")
        self.reset_button = QPushButton("Reset")

        grid.addWidget(self.participant_number_text, 0,0)
        grid.addWidget(self.store_button, 1,0)

        for i, method in enumerate(self.methods_labels):
            radio = QRadioButton(method)
            self.methods_radio_buttons.append(radio)
            grid.addWidget(radio, 3+i, 0)

        grid.addWidget(self.reset_button, 7,0)

        groupBox.setLayout(grid)

        self.grid_rating_sheet.addWidget(groupBox)

    def createRatingSheet(self):
        for i,label in enumerate(self.rating_labels_with_description):
            self.grid_rating_sheet.addWidget(self.createSlider(label), i, 0)

    def createComparisonCards(self):

        for i, pair in enumerate(self.comparison_pairs):
            groupbox = QGroupBox()

            comparison_radio_hbox = QHBoxLayout()
            
            radio1 = QRadioButton(pair[0])
            radio2 = QRadioButton(pair[1])

            self.comparison_pairs_radio_buttons.append([radio1, radio2])

            comparison_radio_hbox.addWidget(radio1)
            comparison_radio_hbox.addWidget(radio2)

            groupbox.setLayout(comparison_radio_hbox)
            
            self.grid_comparison_cards.addWidget(groupbox, i, 1)

    def createSlider(self, label):
        groupBox = QGroupBox("")
        label1 = QLabel(label)

        self.label = QLabel(alignment=Qt.AlignCenter)

        slider = Slider(tickPosition=QSlider.TicksLeft,
            orientation=Qt.Horizontal)
            
        slider_vbox = QVBoxLayout()
        slider_hbox = QHBoxLayout()
        slider_hbox.setContentsMargins(0, 0, 0, 0)
        slider_vbox.setContentsMargins(0, 0, 0, 0)
        slider_vbox.setSpacing(0)

        label_minimum = QLabel(alignment=Qt.AlignLeft)
        slider.minimumChanged.connect(label_minimum.setNum)
        label_maximum = QLabel(alignment=Qt.AlignRight)
        slider.maximumChanged.connect(label_maximum.setNum)
        slider_vbox.addWidget(slider)
        slider_vbox.addLayout(slider_hbox)
        slider_hbox.addWidget(label_minimum, Qt.AlignLeft)
        slider_hbox.addWidget(label_maximum, Qt.AlignRight)
        slider_vbox.addStretch()

        slider.setMinimum(0)
        slider.setMaximum(100)
        slider.setValue(choice([0, 20, 40, 60, 80, 100]))
        slider.setTickInterval(5)
        slider.setSingleStep(100/20)

        vbox = QVBoxLayout(self)
        vbox.addWidget(label1)
        vbox.addLayout(slider_vbox)
        vbox.addWidget(self.label)
        self.setGeometry(300, 300, 300, 150)
        slider.valueChanged.connect(self.label.setNum)

        self.rating_sliders.append(slider)

        groupBox.setLayout(vbox)

        return groupBox

if __name__ == '__main__':
    app = QApplication(sys.argv)
    clock = NASATLX()
    clock.show()
    sys.exit(app.exec_())