from HorizonDetector import HorizonDetector
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
        #frame = cv2.imread("nav_algo\computer_vision\detectors\red-lighthouse-in-cayuga-lake-new-york-mingqi-ge.jpg")
        # scale image
        max_dimension = max(frame.shape)
        scale = 700 / max_dimension
        frame = cv2.resize(frame, None, fx=scale, fy=scale)
        bd = HorizonDetector()
        bd.process(frame)

        lines = bd.find_lines_output
        found = bd.find_lines_output != None
        lines = bd.filter_lines_output
        found = lines != None
        color = (0,255,0)

        for i in lines:
            pt1 = (int(i.x1), int(i.y1))
            pt2 = (int(i.x2), int(i.y2))
            cv2.line(frame, pt1, pt2, color, 3)

        cv2.imshow('horizon detection', frame)

        # press q to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            exit()


if __name__ == '__main__':
    main()
