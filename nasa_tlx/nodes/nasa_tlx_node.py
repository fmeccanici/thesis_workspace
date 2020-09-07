#!/usr/bin/env python3.5 

import rospy, sys
from PyQt5.QtWidgets import QApplication

from nasa_tlx_python.nasa_tlx_python import NASATLX

class NasaTlxNode():
    def __init__(self):
        rospy.init_node('nasa_tlx')
    
    def run(self):
        app = QApplication(sys.argv)
        clock = NASATLX()
        clock.show()
        sys.exit(app.exec_())
if __name__ == "__main__":
    node = NasaTlxNode()
    node.run() 