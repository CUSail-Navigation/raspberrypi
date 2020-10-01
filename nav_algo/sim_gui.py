import nav_algo.boat as boat
from nav_algo.navigation_helper import *
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
        self.waypoint = coord.Vector(x=1, y=1)  # TODO make dynamic

        pg.setConfigOption('background', 'w')
        self.app = QtGui.QApplication(sys.argv)  # init Qt
        self.w = QtGui.QWidget()  # top level widge to hold everything

        # exit cleaner
        exit_action = QtGui.QAction("Exit", self.app)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(lambda: exit(0))

        ## Display the widget as a new window
        self.w.setWindowTitle("CUSail Simulator")
        self.w.show()

        self.run()  # testing only

        self.app.exec_()  # start GUI in a new thread

    def startEventAlgo(self):
        # TODO link this to a button on the GUI and do other stuff
        self.w.timer = QTimer()
        self.w.timer.setInterval(100)  # 10 times a second for now?
        self.w.timer.timeout.connect(self.run)
        self.w.timer.start()

    def run(self):
        self.boatController.updateSensors()
        intended_angle = newSailingAngle(self.boatController, self.waypoint)
        sail_angle, tail_angle = self.boatController.getServoAngles(
            intended_angle)

        print("intended angle: {}, sail: {}, tail {}".format(
            intended_angle, sail_angle, tail_angle))
