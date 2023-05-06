from nav_algo.computer_vision.detectors.buoyDetector import BuoyDetector
from nav_algo.computer_vision.detectors.boatDetector import BoatDetector
import nav_algo.coordinates as coord
import cv2
from picamera2 import *


class Camera:
    def __init__(self):
        self.camera = Picamera2()
        self.camera.start_preview(Preview.NULL)

        # This by default swaps blue and red relative to what opencv is expecting,
        # we have to set the configuration to fix that. If for some reason you see
        # blue and red swapped, remove the "format" parameter from the line below.
        config = self.camera.create_preview_configuration(main={"size": (640, 480), "format": 'XRGB8888'})
        self.camera.configure(config)
        self.camera.start()

        self.buoyDetector = BuoyDetector()
        self.boatDetector = BoatDetector()

        self.get_image()

    def get_image(self):
        self.img_array = self.camera.capture_array()
    
    def read_buoy(self, yaw, curr_x, curr_y):
        self.get_image()
        frame = self.img_array

        max_dimension = max(frame.shape)
        scale = 700 / max_dimension
        frame = cv2.resize(frame, None, fx=scale, fy=scale)

        self.buoyDetector.process(frame)

        buoyCoordsTuple = self.buoyDetector.get_buoy_coords(
            yaw, curr_x, curr_y)
            
        if buoyCoordsTuple is not None:
            buoyCoords = coord.Vector(x=buoyCoordsTuple[0], y=buoyCoordsTuple[1])
        else:
            buoyCoords = None

        return buoyCoords

    def read_boat(self, yaw, curr_x, curr_y):
        self.get_image()
        frame = self.img_array

        max_dimension = max(frame.shape)
        scale = 700 / max_dimension
        frame = cv2.resize(frame, None, fx=scale, fy=scale)

        self.boatDetector.process(frame)

        boatCoordsTuple = self.boatDetector.get_boat_coords(
            yaw, curr_x, curr_y)
            
        if boatCoordsTuple is not None:
            boatCoords = coord.Vector(x=boatCoordsTuple[0], y=boatCoordsTuple[1])
        else:
            boatCoords = None

        return boatCoords
