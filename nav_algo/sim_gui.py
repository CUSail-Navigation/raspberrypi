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

        pg.setConfigOption('background', 'w')
        self.app = QtGui.QApplication(sys.argv)  # init Qt
        self.w = QtGui.QWidget()  # top level widge to hold everything

        # exit cleaner
        exit_action = QtGui.QAction("Exit", self.app)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(lambda: exit(0))

        ## Display the widget as a new window
        self.w.setWindowTitle("CUSail Simulator")
        self.layout = QtGui.QGridLayout()
        self.w.setLayout(self.layout)

        # widgets
        self.windDirLab = QtGui.QLabel('Wind Direction (wrt East):')
        self.windDirIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.layout.addWidget(self.windDirLab, 0, 0)
        self.layout.addWidget(self.windDirIn, 0, 1)

        self.windSpeedLab = QtGui.QLabel('Wind Speed (m/s):')
        self.windSpeedIn = QtGui.QSpinBox(maximum=100, minimum=0)
        self.layout.addWidget(self.windSpeedLab, 1, 0)
        self.layout.addWidget(self.windSpeedIn, 1, 1)

        self.rollLab = QtGui.QLabel('Roll:')
        self.rollIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.layout.addWidget(self.rollLab, 2, 0)
        self.layout.addWidget(self.rollIn, 2, 1)

        self.pitchLab = QtGui.QLabel('Pitch:')
        self.pitchIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.layout.addWidget(self.pitchLab, 3, 0)
        self.layout.addWidget(self.pitchIn, 3, 1)

        self.yawLab = QtGui.QLabel('Yaw (wrt East):')
        self.yawIn = QtGui.QSpinBox(maximum=359, minimum=0)
        self.layout.addWidget(self.yawLab, 4, 0)
        self.layout.addWidget(self.yawIn, 4, 1)

        self.targetLab = QtGui.QLabel('Target (x, y):')
        self.targetXIn = QtGui.QSpinBox(minimum=-1000)
        self.targetYIn = QtGui.QSpinBox(minimum=-1000)
        self.layout.addWidget(self.targetLab, 5, 0)
        self.layout.addWidget(self.targetXIn, 5, 1)
        self.layout.addWidget(self.targetYIn, 5, 2)

        self.posLab = QtGui.QLabel('Position (x, y):')
        self.posXIn = QtGui.QSpinBox(minimum=-1000)
        self.posYIn = QtGui.QSpinBox(minimum=-1000)
        self.layout.addWidget(self.posLab, 6, 0)
        self.layout.addWidget(self.posXIn, 6, 1)
        self.layout.addWidget(self.posYIn, 6, 2)

        self.calcButton = QtGui.QPushButton('Run Algorithm')
        self.layout.addWidget(self.calcButton, 7, 0)
        self.calcButton.clicked.connect(self.runNavAlgo)

        self.intAngLab = QtGui.QLabel('Intended Angle: --')
        self.layout.addWidget(self.intAngLab, 0, 4)

        self.sailAngLab = QtGui.QLabel('Sail Angle: --')
        self.layout.addWidget(self.sailAngLab, 1, 4)

        self.tailAngLab = QtGui.QLabel('Tail Angle: --')
        self.layout.addWidget(self.tailAngLab, 2, 4)

        # TODO add a dropdown for event types
        # TODO add something to take in the mock sensor file name
        # TODO add a button to call startEventAlgo

        self.w.show()

        self.app.exec_()  # start GUI in a new thread

    def runEventAlgo(self):
        # TODO mock the sensor readings
        # TODO call the nav helper navigate function to move one step forward
        # TODO pass boat config to physics engine, update gui
        # TODO use physics engine output to update boat position.
        pass

    def runNavAlgo(self):
        # self.boatController.updateSensors()
        self.boatController.sensors.position = coord.Vector(
            x=self.posXIn.value(), y=self.posYIn.value())
        self.waypoint = coord.Vector(x=self.targetXIn.value(),
                                     y=self.targetYIn.value())

        self.boatController.sensors.wind_direction = self.windDirIn.value()
        self.boatController.sensors.wind_speed = self.windSpeedIn.value()

        self.boatController.sensors.roll = self.rollIn.value()
        self.boatController.sensors.pitch = self.pitchIn.value()
        self.boatController.sensors.yaw = self.yawIn.value()

        intended_angle = newSailingAngle(self.boatController, self.waypoint)
        sail_angle, tail_angle = self.boatController.getServoAngles(
            intended_angle)

        self.intAngLab.setText('Intended Angle: ' + str(intended_angle))
        self.sailAngLab.setText('Sail Angle: ' + str(sail_angle))
        self.tailAngLab.setText('Tail Angle: ' + str(tail_angle))

    def startEventAlgo(self):
        # TODO get type of event from the dropdown menu
        # TODO get name of mock sensor file (maybe from a text box on gui)
        # TODO call the event function in nav helper
        # TODO call runEventAlgo every 0.1s(?) - use a QTimer
        pass