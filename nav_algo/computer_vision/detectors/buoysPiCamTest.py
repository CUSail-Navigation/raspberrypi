from buoyDetectorPi import BuoyDetector
import cv2
import numpy as np
import time
from picamera2 import *


def start():
    """Sets initial values of the camera, then begins loop for running the 
    detector."""

    camera = Picamera2()
    camera.start_preview(Preview.NULL)

    # This by default swaps blue and red relative to what opencv is expecting,
    # we have to set the configuration to fix that. If for some reason you see
    # blue and red swapped, remove the "format" parameter from the line below.
    config = camera.create_preview_configuration(main={"size": (640, 480), "format": 'XRGB8888'})
    camera.configure(config)
    camera.start()

    print("Press q to quit.")

    while True:
        im = camera.capture_array()
        run(im)


def run(frame):
    """Creates a detector object that proesses camera feed."""

    max_dimension = max(frame.shape)
    scale = 700 / max_dimension
    frame = cv2.resize(frame, None, fx=scale, fy=scale)

    bd = BuoyDetector()
    bd.process(frame)

    contours = bd.find_contours_output
    found = bd.find_contours_output != None

    contours = bd.filter_contours_output
    found = contours != None

    print(bd.find_distance_largest_contour())

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    cv2.imshow("buoy detection", frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        exit(0)


if __name__ == '__main__':
    start()
    
