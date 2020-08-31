from itertools import cycle
from PyQt5 import QtCore, QtGui, QtWidgets

class TrafficLight(QtWidgets.QMainWindow):
    def __init__(self,parent = None):
        super(TrafficLight, self).__init__(parent)
        self.setWindowTitle("TrafficLight ")
        self._current_color = QtGui.QColor('red')

        self.resize(200, 400)

    @QtCore.pyqtSlot()
    def set_color(self, color):
        self._current_color = QtGui.QColor(color)
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setBrush(self._current_color)
        p.setPen(QtCore.Qt.black)
        p.drawEllipse(self.rect().center(), 50, 50)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = TrafficLight()
    w.show() 
    sys.exit(app.exec_())