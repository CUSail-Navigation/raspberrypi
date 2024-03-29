import cv2
import numpy
import math
from enum import Enum


class HorizonDetector:
    BlurType = Enum('BlurType',
                    'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

    def __init__(self):
        """
        Initializes all values to presets or None if need to be set
        """
        self.__blur_type = HorizonDetector.BlurType.Box_Blur
        self.__blur_radius = 3.30188679245283

        self.blur_output = None

        self.__find_lines_input = self.blur_output

        self.find_lines_output = None

        self.__filter_lines_lines = self.find_lines_output
        self.__filter_lines_min_length = 175.0
        self.__filter_lines_angle = [-25, 25]

        self.filter_lines_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step Blur0:
        self.__blur_input = source0
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type,
                                         self.__blur_radius)

        # Step Find_Lines0:
        self.__find_lines_input = self.blur_output
        (self.find_lines_output) = self.__find_lines(self.__find_lines_input)

        # Step Filter_Lines0:
        self.__filter_lines_lines = self.find_lines_output
        (self.filter_lines_output) = self.__filter_lines(
            self.__filter_lines_lines, self.__filter_lines_min_length,
            self.__filter_lines_angle)

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src (numpy.ndarray): The source mat.
            type (int): The blurType to perform represented as an int.
            radius (float): The radius for the blur as a float.
        Returns:
            numpy.ndarray: A matrix that has been blurred.
        """
        if (type is HorizonDetector.BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif (type is HorizonDetector.BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif (type is HorizonDetector.BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    class Line:
        def __init__(self, x1, y1, x2, y2):
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2

        def length(self):
            return numpy.sqrt(
                pow(self.x2 - self.x1, 2) + pow(self.y2 - self.y1, 2))

        def angle(self):
            return math.degrees(
                math.atan2(self.y2 - self.y1, self.x2 - self.x1))

    @staticmethod
    def __find_lines(input):
        """Finds all line segments in an image.
        Args:
            input (numpy.ndarray): A numpy.ndarray.
        Returns:
            list: A filtered list of Lines.
        """
        detector = cv2.createLineSegmentDetector()
        if (len(input.shape) == 2 or input.shape[2] == 1):
            lines = detector.detect(input)
        else:
            tmp = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
            lines = detector.detect(tmp)
        output = []
        if len(lines) != 0:
            for i in range(1, len(lines[0])):
                tmp = HorizonDetector.Line(lines[0][i, 0][0], lines[0][i,
                                                                       0][1],
                                           lines[0][i, 0][2], lines[0][i,
                                                                       0][3])
                output.append(tmp)
        return output

    @staticmethod
    def __filter_lines(inputs, min_length, angle):
        """Filters out lines that do not meet certain criteria.
        Args:
            inputs (list): A list of Lines.
            min_Lenght (float): The minimum lenght that will be kept.
            angle (list): The minimum and maximum angles in degrees as a list of two numbers.
        Returns:
            list: A filtered list of Lines.
        """
        outputs = []
        for line in inputs:
            if (line.length() > min_length):
                if (line.angle() >= angle[0] and line.angle() <= angle[1]):
                        # or (line.angle() + 180.0 >= angle[0]
                        #     and line.angle() + 180.0 <= angle[1])):
                    outputs.append(line)
        return outputs
