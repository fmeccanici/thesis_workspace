from itertools import cycle
from PyQt5 import QtCore, QtGui, QtWidgets

class TrafficLightUpdater(object):
    def __init__(self, text_path='/home/fmeccanici/Documents/thesis/thesis_workspace/src/data_logger/', text_file='light.txt'):
        self.text_path = text_path
        self.text_file = text_path + text_file
    
    def update(self, light):
        with open(self.text_file, 'w') as f:
            f.write(light)