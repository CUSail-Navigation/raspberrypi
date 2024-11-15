import unittest
import sys
import nav_algo.basic_algo as algo
import nav_algo.coordinates as coord


class TestBasicAlgo(unittest.TestCase):
    def test_normalize_angle(self):
        b_algo = algo.BasicAlgo
        self.assertAlmostEqual(b_algo.normalize_angle(45), 45)
        self.assertAlmostEqual(b_algo.normalize_angle(480), 120)

    def calibrate_wind_direction(self):
        b_algo = algo.BasicAlgo
        self.assertAlmostEqual(b_algo.calibrate_wind_direction(300, 80), 220)
        self.assertAlmostEqual(b_algo.calibrate_wind_direction(50, 100), 50)

    def test_setSail(self):
        b_algo = algo.BasicAlgo
        self.assertAlmostEqual(b_algo.setSail(15, 10), 90)
        self.assertAlmostEqual(b_algo.setSail(359, 5), 90)
        self.assertAlmostEqual(b_algo.setSail(350, 20), 75)
        self.assertAlmostEqual(b_algo.setSail(90, 0), -45)
        self.assertAlmostEqual(b_algo.setSail(0, 90), 45)
        self.assertAlmostEqual(b_algo.setSail(180, 0), 0)

    def test_setRudder(self):
        b_algo = algo.BasicAlgo
        currLoc = coord.Vector(x=0, y=0)
        tackingPoint = (0, 1)
        headingDir = 20
        currDest = coord.Vector(x=1, y=1)

        # tacking is True, the boat will sail to tackingPoint instead of currDest
        self.assertAlmostEqual(b_algo.setRudder(currLoc, True, tackingPoint,
        headingDir, currDest), -20)

        tackingPoint = (-1, 0)
        self.assertAlmostEqual(b_algo.setRudder(currLoc, True, tackingPoint,
        headingDir, currDest), -40)

        tackingPoint = (0, -1)
        self.assertAlmostEqual(b_algo.setRudder(currLoc, True, tackingPoint,
        headingDir, currDest), 30)

        # tacking is False, the boat will sail to currDest instead of tackingPoint
        self.assertAlmostEqual(b_algo.setRudder(currLoc, False, tackingPoint,
        headingDir, currDest), -5)

        currDest = coord.Vector(x=0, y=1)
        self.assertAlmostEqual(b_algo.setRudder(currLoc, False, tackingPoint,
        headingDir, currDest), -20)
        
        currDest = coord.Vector(x=-1, y=0)
        self.assertAlmostEqual(b_algo.setRudder(currLoc, False, tackingPoint,
        headingDir, currDest), -40)

        currDest = coord.Vector(x=0, y=-1)
        self.assertAlmostEqual(b_algo.setRudder(currLoc, False, tackingPoint,
        headingDir, currDest), 30)
    
    def test_calculateTP(self):
        b_algo = algo.BasicAlgo
        currentLocation = coord.Vector(x=0, y=0)
        destination = coord.Vector(x=0, y=1)
        windDirection = 210
        headingDirection = 0
        print(b_algo.calculateTP(currentLocation, destination, windDirection, headingDirection))
        self.assertAlmostEqual((), b_algo.calculateTP(currentLocation, destination, windDirection, headingDirection))

#create some object
if __name__ == '__main__':
    unittest.main()