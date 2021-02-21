import cv2
import numpy as np

green = (0, 255, 0)


def overlay_mask(mask, image):
    #make the mask rgb
    rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    img = cv2.addWeighted(rgb_mask, 0.5, image, 0.5, 0)
    return img


def find_biggest_contour(image):
    #get the contours
    image, contours, _ = cv2.findContours(image.copy(), cv2.RETR_LIST,
                                          cv2.CHAIN_APPROX_SIMPLE)

    #isolating the largest contour
    contour_sizes = [(cv2.contourArea(contour), contour)
                     for contour in contours]

    if len(contour_sizes) == 0:
        return None, None

    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

    mask = np.zeros(image.shape, np.uint8)
    cv2.drawContours(mask, [biggest_contour], -1, 255, -1)

    return biggest_contour, mask


def find_obj(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    #clean the image
    image_blur = cv2.GaussianBlur(image, (7, 7), 0)
    image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

    #thresholding
    #color
    min_red1 = np.array([0, 100, 80])
    max_red1 = np.array([10, 256, 256])
    mask1 = cv2.inRange(image_blur_hsv, min_red1, max_red1)

    #brightness
    min_red2 = np.array([170, 100, 80])
    max_red2 = np.array([180, 256, 256])
    mask2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)

    #combine masks
    mask = mask1 + mask2

    #segmentation - https://docs.opencv.org/trunk/d9/d61/tutorial_py_morphological_ops.html
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    # gets rid of small dots within an object
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # gets rid of small dots outside of an object
    mask_clean = cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

    #find the largest object
    big_obj_contour, mask_obj = find_biggest_contour(mask_clean)
    if big_obj_contour is None:
        return False, None

    #check the size of the contour (guess 500 is too small)
    area = cv2.contourArea(big_obj_contour)
    if area < 500:
        return False, cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    #overlay the mask on the image
    overlay = overlay_mask(mask_clean, image)

    #circle the biggest one
    cv2.drawContours(overlay, big_obj_contour, -1, green, 5)

    return True, cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)


vid = cv2.VideoCapture(0)

print("Press q to quit.")
while (True):
    _, frame = vid.read()  # get a frame from the webcam

    #scale image
    max_dimension = max(frame.shape)
    scale = 700 / max_dimension
    frame = cv2.resize(frame, None, fx=scale, fy=scale)

    found, result_img = find_obj(frame)
    if found:
        cv2.imshow('red detection', result_img)
    else:
        cv2.imshow('red detection', frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
