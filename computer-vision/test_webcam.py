import cv2

vid = cv2.VideoCapture(0)

print("Press q to quit.")
while (True):
    _, frame = vid.read()  # get a frame from the webcam
    cv2.imshow('webcam feed', frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()  # release the capture object
cv2.destroyAllWindows()
