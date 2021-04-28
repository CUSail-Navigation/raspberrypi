import cv2
from enum import Enum
#from nav_algo.computer_vision.detectors.utils import find_distances, get_coords


class BuoyDetector:
    """A detector for buoys. 
    
    This uses a standard webcam to output visual feedback on the camera screen 
    and an outline of the buoys. It returns the coordinates of the largest buoy 
    found.

    Args:
        img_height: the height of the camera output.
        img_width: the width of the camera output.
    """
    BUOY_HEIGHT = 1016  # buoy's height in mm
    BlurType = Enum('BlurType',
                    'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

    def __init__(self, img_height=480, img_width=640):
        """
        Initializes all values to presets or None if need to be set
        """
        self.img_height = img_height
        self.img_width = img_width

        self.__rgb_threshold_red = [100, 255.0]
        self.__rgb_threshold_green = [100, 200]
        self.__rgb_threshold_blue = [0, 100]

        self.rgb_threshold_output = None

        self.__cv_erode_src = self.rgb_threshold_output
        self.__cv_erode_kernel = None
        self.__cv_erode_anchor = (-1, -1)
        self.__cv_erode_iterations = 1.0
        self.__cv_erode_bordertype = cv2.BORDER_CONSTANT
        self.__cv_erode_bordervalue = (-1)

        self.cv_erode_output = None

        self.__blur_input = self.cv_erode_output
        self.__blur_type = BuoyDetector.BlurType.Box_Blur
        self.__blur_radius = 21

        self.blur_output = None

        self.__cv_dilate_src = self.blur_output
        self.__cv_dilate_kernel = None
        self.__cv_dilate_anchor = (-1, -1)
        self.__cv_dilate_iterations = 7.0
        self.__cv_dilate_bordertype = cv2.BORDER_CONSTANT
        self.__cv_dilate_bordervalue = (-1)

        self.cv_dilate_output = None

        self.__find_contours_input = self.cv_dilate_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 0.0
        self.__filter_contours_min_perimeter = 0.0
        self.__filter_contours_min_width = 0.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 0.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [0, 100]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 0.0
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

        self.filter_contours_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step RGB_Threshold0:
        self.__rgb_threshold_input = source0
        (self.rgb_threshold_output) = self.__rgb_threshold(
            self.__rgb_threshold_input, self.__rgb_threshold_red,
            self.__rgb_threshold_green, self.__rgb_threshold_blue)

        # Step CV_erode0:
        self.__cv_erode_src = self.rgb_threshold_output
        (self.cv_erode_output) = self.__cv_erode(self.__cv_erode_src,
                                                 self.__cv_erode_kernel,
                                                 self.__cv_erode_anchor,
                                                 self.__cv_erode_iterations,
                                                 self.__cv_erode_bordertype,
                                                 self.__cv_erode_bordervalue)

        # Step Blur0:
        self.__blur_input = self.cv_erode_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type,
                                         self.__blur_radius)

        # Step CV_dilate0:
        self.__cv_dilate_src = self.blur_output
        (self.cv_dilate_output) = self.__cv_dilate(
            self.__cv_dilate_src, self.__cv_dilate_kernel,
            self.__cv_dilate_anchor, self.__cv_dilate_iterations,
            self.__cv_dilate_bordertype, self.__cv_dilate_bordervalue)

        # Step Find_Contours0:
        self.__find_contours_input = self.cv_dilate_output
        (self.find_contours_output) = self.__find_contours(
            self.__find_contours_input, self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(
            self.__filter_contours_contours, self.__filter_contours_min_area,
            self.__filter_contours_min_perimeter,
            self.__filter_contours_min_width, self.__filter_contours_max_width,
            self.__filter_contours_min_height,
            self.__filter_contours_max_height, self.__filter_contours_solidity,
            self.__filter_contours_max_vertices,
            self.__filter_contours_min_vertices,
            self.__filter_contours_min_ratio, self.__filter_contours_max_ratio)

    @staticmethod
    def __rgb_threshold(input, red, green, blue):
        """Segment an image based on color ranges.

        Args:
            input (numpy.ndarray): A BGR numpy.ndarray.
            red (list): A list of two numbers the are the min and max red.
            green (list): A list of two numbers the are the min and max green.
            blue (list): A list of two numbers the are the min and max blue.

        Returns:
            numpy.ndarray: A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
        return cv2.inRange(out, (red[0], green[0], blue[0]),
                           (red[1], green[1], blue[1]))

    @staticmethod
    def __cv_erode(src, kernel, anchor, iterations, border_type, border_value):
        """Expands area of lower value in an image.

        Args:
           src (numpy.ndarray): A numpy.ndarray.
           kernel (numpy.ndarray): The kernel for erosion. A numpy.ndarray.
           iterations (int): the number of times to erode.
           border_type (Enum): Opencv enum that represents a border type.
           border_value (int): value to be used for a constant border.

        Returns:
            numpy.ndarray: A numpy.ndarray after erosion.
        """
        return cv2.erode(src,
                         kernel,
                         anchor,
                         iterations=(int)(iterations + 0.5),
                         borderType=border_type,
                         borderValue=border_value)

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.

        Args:
            src (numpy.ndarray): The source mat.
            type (int): The blurType to perform represented as an int.
            radius (float): The radius for the blur as a float.

        Returns:
            numpy.ndarray: A numpy.ndarray that has been blurred.
        """
        if (type is BuoyDetector.BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif (type is BuoyDetector.BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif (type is BuoyDetector.BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __cv_dilate(src, kernel, anchor, iterations, border_type,
                    border_value):
        """Expands area of higher value in an image.

        Args:
           src (numpy.ndarray): A numpy.ndarray.
           kernel (numpy.ndarray): The kernel for dilation. A numpy.ndarray.
           iterations (int): the number of times to dilate.
           border_type (Enum): Opencv enum that represents a border type.
           border_value (int): value to be used for a constant border.

        Returns:
            numpy.ndarray: A numpy.ndarray after dilation.
        """
        return cv2.dilate(src,
                          kernel,
                          anchor,
                          iterations=(int)(iterations + 0.5),
                          borderType=border_type,
                          borderValue=border_value)

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the 
        nearest black pixel.

        Args:
            input (numpy.ndarray): A numpy.ndarray.
            external_only (bool): A boolean. If true only external contours are 
            found.

        Returns:
            list: A list of numpy.ndarray where each one represents a contour.
        """
        if (external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width,
                          max_width, min_height, max_height, solidity,
                          max_vertex_count, min_vertex_count, min_ratio,
                          max_ratio):
        """Filters out contours that do not meet certain criteria.

        Args:
            input_contours (list): Contours as a list of numpy.ndarray.
            min_area (float): The minimum area of a contour that will be kept.
            min_perimeter (float): The minimum perimeter of a contour that will 
            be kept.
            min_width (float): Minimum width of a contour.
            max_width (float): MaxWidth maximum width.
            min_height (float): Minimum height.
            max_height (float): Maximimum height.
            solidity (list): The minimum and maximum solidity of a contour.
            min_vertex_count (int): Minimum vertex Count of the contours.
            max_vertex_count (int): Maximum vertex Count.
            min_ratio (float): Minimum ratio of width to height.
            max_ratio (float): Maximum ratio of width to height.

        Returns:
            listContours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count
                    or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output

    # def find_distances(self):
    #     """Calculates distances from each contour and creates list of obstacle 
    #     distances from camera.

    #     Args:
    #         img_height: height of image passed in, in pixels.

    #     Returns:
    #         list: A list where each element represents an obstacle distance.
    #         list: A list where each element represents an x-offset in the image.
    #     """
    #     return find_distances(self.filter_contours_output, self.img_height,
    #                           self.img_width, BuoyDetector.BUOY_HEIGHT)

    # def get_buoy_coords(self, direction, curr_x, curr_y):
    #     """Calculates all visible buoy coordinates.

    #     Args:
    #         direction (float): Boat's current direction (an angle)
    #         curr_x (float): Boat's current x-coordinate
    #         curr_y (float): Boat's current y-coordinate
    
    #     Returns:
    #         list: A list of coordinate pairs (x, y) representing the center of 
    #         each detected buoy.
    #     """
    #     coord_list = []
    #     dists, x_offsets = find_distances
    #     for d, x in zip(dists, x_offsets):
    #         coord_list.append(get_coords(d, x, direction, curr_x, curr_y))
    #     return coord_list
