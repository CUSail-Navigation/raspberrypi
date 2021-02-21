from detectors.buoyDetector.buoyDetector import BuoyDetector
import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

print("Press q to quit.")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = frame.array

    max_dimension = max(frame.shape)
    scale = 700 / max_dimension
    frame = cv2.resize(frame, None, fx=scale, fy=scale)

    bd = BuoyDetector()
    bd.process(frame)

    contours = bd.filter_contours_output
    found = contours != None

    print(bd.find_distances(frame.shape[0], frame.shape[1]))

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    cv2.imshow('buoy detection', frame)

    rawCapture.truncate(0)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
