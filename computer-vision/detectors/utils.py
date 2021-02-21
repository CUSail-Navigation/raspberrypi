import cv2
import math

# Constants (could maybe double-check these)
SENSOR_HEIGHT = 2.74
FOCAL_LENGTH = 3.60  # focal length of raspberry pi cam 1

"""Calculates distances from each contour and creates list of obstacle distances from camera.
Args:
  img_height: height of image passed in, in pixels
Return:
  A list where each element represents an obstacle distance in meters.
"""


def find_distances(contours_output, img_height, img_width, obstacle_width):

    distances = []
    x_displacements = []

    for contour in contours_output:
        center, size, angle = cv2.minAreaRect(contour)
        width, height = size
        distances.append((obstacle_width * FOCAL_LENGTH * img_height /
                          (height * SENSOR_HEIGHT)) / 1000)
        x_displacements.append(center - img_width / 2)

    return distances, x_displacements


'''
get_coord(distance, x_displacement, direction, curr_x, curr_y) returns the
x and y coordinates of the center of an obstacle given a calculated [distance] in front
of the boat at coordinates [curr_x], [curr_y] facing [direction]

Return:
  A pair of coordinates x, y representing obstacle center.
'''


def get_coords(distance, x_displacement, direction, curr_x, curr_y):

    # TODO: Convert based on x displacement

    buoy_x = curr_x + distance * math.cos(direction)
    buoy_y = curr_y + distance * math.sin(direction)

    # camera facing same direction as boat
    return buoy_x, buoy_y  # returns one buoy's coordinates in our coordinate system
