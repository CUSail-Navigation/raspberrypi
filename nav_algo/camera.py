from nav_algo.computer_vision.detectors.buoyDetector.buoyDetector import BuoyDetector
from nav_algo.computer_vision.detectors.boatDetector import BoatDetector
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera


class Camera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32

        self.buoyDetector = BuoyDetector()
        self.boatDetector = BoatDetector()

        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))

    def read(self, direction, curr_x, curr_y):
        for frame in self.camera.capture_continuous(self.rawCapture,
                                                    format="bgr",
                                                    use_video_port=True):

            frame = frame.array
            max_dimension = max(frame.shape)
            scale = 700 / max_dimension
            frame = cv2.resize(frame, None, fx=scale, fy=scale)

            self.buoyDetector.process(frame)
            self.boatDetector.process(frame)

            buoyCoordsTuple = self.buoyDetector.get_buoy_coords(
                direction, curr_x, curr_y)
            boatCoordsTuple = self.boatDetector.get_boat_coords(
                direction, curr_x, curr_y)

            self.rawCapture.truncate(0)

            buoyCoords = coord.Vector(x=buoyCoordsTuple[0], y=buoyCoordsTuple[1])
            boatCoords = coord.Vector(x=boatCoordsTuple[0], y=boatCoordsTuple[1])

            return buoyCoords, boatCoords
