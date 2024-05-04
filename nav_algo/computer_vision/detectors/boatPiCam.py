from ultralytics import YOLO
import cv2
import math 
import numpy as np
import time
from picamera2 import *

# model
model = YOLO("yolov8m.pt")
# model.train(data='coco128.yaml', epochs=3)  # train the model


# object classes
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]


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

    results = model(frame, stream=True)

    # coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

            # put box in cam
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])
            if classNames[cls] == "boat":
                # print(str(x1) + ", " + str(y1))
                xaxis = x1 + x2 // 2 
                yaxis = y1 + y2 // 2

                if xaxis > 640/2: 
                    return "right"
                else:
                    return "left"
                # camera is flipped so should turn right when its on left side??
                # maybe swap these


            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(frame, classNames[cls], org, font, fontScale, color, thickness)

    cv2.imshow("boat detection", frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        exit(0)


if __name__ == '__main__':
    start()
    