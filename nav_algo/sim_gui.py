import nav_algo.boat as boat
import sys
from PyQt5 import QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication, QWidget, QSpinBox
import pyqtgraph as pg
import time
import numpy as np


class GUI:
    def __init__(self, boatController: boat.BoatController):
        self.boatController = boatController

        pg.setConfigOption('background', 'w')
        app = QtGui.QApplication(sys.argv)  # init Qt
        w = QtGui.QWidget()  # top level widge to hold everything

        # exit cleaner
        exit_action = QtGui.QAction("Exit", app)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(lambda: exit(0))

        ## Display the widget as a new window
        w.setWindowTitle("CUSail Simulator")
        w.show()
        app.exec_()  # start GUI in a new thread
