import cv2
import numpy as np

targets = [
    ("Image 1", cv2.imread("img/1.jpg"), 4, 4),
    ("Image 2", cv2.imread("img/2.jpg"), 2, 4),
    ("Image 3", cv2.imread("img/3.jpg"), 5, 8)
]

for target in targets:
    name = target[0]
    img = target[1]
    minFind = target[2]
    maxFind = target[3]

    found = -1

    # convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # threshold
    lower_orange = np.array([0,186,144])
    upper_orange = np.array([62,255,255])
    threshold = cv2.inRange(hsv, lower_orange, upper_orange)

    # reduce noise
    kernel = np.ones((9,9),np.uint8)
    opening = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
    cv2.imshow('open', opening)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # find contours
    contours,hierarchy = cv2.findContours(opening, cv2.RETR_EXTERNAL, 2)
    found = len(contours)

    if found < minFind or found > maxFind:
        print("FAILED :( " + name + " found " + str(found))
    else:
        print("PASSED :D " + name)
