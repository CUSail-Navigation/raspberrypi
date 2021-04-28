#from .nav_algo.computer_vision.detectors.buoyDetector.buoyDetector import BuoyDetector
from buoyDetector import BuoyDetector
import cv2
import numpy as np
import time


def main():
    """Sets initial values of the camera, then begins loop for running the 
    detector, which creates a detector object that proesses camera feed."""
    vid = cv2.VideoCapture(0)

    print("Press q to quit.")
    while (True):
        _, frame = vid.read()  # get a frame from the webcam
        # scale image
        max_dimension = max(frame.shape)
        scale = 700 / max_dimension
        frame = cv2.resize(frame, None, fx=scale, fy=scale)
        bd = BuoyDetector()
        bd.process(frame)

        contours = bd.find_contours_output
        found = bd.find_contours_output != None
        contours = bd.filter_contours_output
        found = contours != None

        #print(bd.get__buoy_coords(0, 0, 0))  # print sample coordinates

        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        cv2.imshow('buoy detection', frame)

        # press q to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break


if __name__ == '__main__':
    main()
