from buoyDetectorPi import BuoyDetector
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

buoyMinSize = 30 # determine good size while testing

def start():
    """Sets initial values of the camera, then begins loop for running the 
    detector."""

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    
    time.sleep(2.0)

    for frame in camera.capture_continuous(rawCapture,
                                           format="bgr",
                                           use_video_port=True):
        run(frame, rawCapture, buoyMinSize)
    

def run(frame, rawCapture, buoyMinSize):
    """Creates a detector object that proesses camera feed."""

    frame = frame.array
    max_dimension = max(frame.shape)
    scale = 700 / max_dimension
    frame = cv2.resize(frame, None, fx=scale, fy=scale)
    bd = BuoyDetector()
    bd.process(frame)

    contours = bd.find_contours_output
    found = bd.find_contours_output != None
    contours = bd.filter_contours_output
    found = contours != None

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    cv2.imshow("buoy detection", frame)
    rawCapture.truncate(0)

    #print(bd.find_distance_largest_contour())
    size, x, y = bd.largest_contour()
    if (size >= buoyMinSize):
        return size, x, y
    else:
        return 0, x, y


if __name__ == '__main__':
    start()
