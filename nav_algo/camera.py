from detectors.buoyDetector.buoyDetector import BuoyDetector
from detectors import BoatDetector
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera


class Camera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32

        self.buoyDetector = BuoyDetector()
        self.boatDetector = BoatDetector()

        rawCapture = PiRGBArray(camera, size=(640, 480))

    def read(direction, curr_x, curr_y):
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

            frame = frame.array
            max_dimension = max(frame.shape)
            scale = 700 / max_dimension
            frame = cv2.resize(frame, None, fx=scale, fy=scale)

            self.buoyDetector.process(frame)
            self.boatDetector.process(frame)

            buoyCoords = self.buoyDetector.get_buoy_coords(
                direction, curr_x, curr_y)
            boatCoords = self.boatDetector.get_boat_coords(
                direction, curr_x, curr_y)

            rawCapture.truncate(0)

            return buoyCoords, boatCoords
