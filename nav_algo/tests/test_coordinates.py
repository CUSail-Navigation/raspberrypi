import unittest
import sys
sys.path.append("..")
import coordinates as coord


upson_hall = (42.444241, 76.481933)
olin_hall = (42.446016, 76.484713)


class TestCoordinatesMethods(unittest.TestCase):
    def test_coordinate_system_init(self):
        coord_sys = coord.CoordinateSystem(upson_hall[0], upson_hall[1])

        self.assertIsInstance(coord_sys, coord.CoordinateSystem)
        self.assertAlmostEqual(coord_sys.LAT_OFFSET, upson_hall[0])
        self.assertAlmostEqual(coord_sys.LONG_OFFSET, upson_hall[1])
        self.assertAlmostEqual(coord_sys.EARTH_RADIUS, 6371000.0)

    def test_vector_init(self):
        coord_sys = coord.CoordinateSystem(upson_hall[0], upson_hall[1])

        # try a lat/long initialization
        vec = coord.Vector(coord_sys, olin_hall[0], olin_hall[1])
        self.assertIsNotNone(vec.coordinate_system)
        self.assertAlmostEqual(vec.latitude, olin_hall[0])
        self.assertAlmostEqual(vec.longitude, olin_hall[1])
        self.assertAlmostEqual(vec.x, 228.1, 1)
        self.assertAlmostEqual(vec.y, 197.37, 2)

        # make sure that the origin is (0,0)
        orig = coord.Vector(coord_sys, upson_hall[0], upson_hall[1])
        self.assertAlmostEqual(orig.x, 0)
        self.assertAlmostEqual(orig.y, 0)

        # make a unit vector with a given angle
        unit_vec = coord.Vector(angle=60)
        self.assertAlmostEqual(unit_vec.x, 0.5)
        self.assertAlmostEqual(unit_vec.y, (3**0.5) / 2)
        self.assertAlmostEqual(unit_vec.magnitude(), 1)

        # make a vector with a given x and y component
        xy_vec = coord.Vector(x=1, y=2)
        self.assertAlmostEqual(xy_vec.x, 1)
        self.assertAlmostEqual(xy_vec.y, 2)

    def test_xyDist(self):
        # test quadrant 1
        p1 = coord.Vector(x=1, y=1)
        p2 = coord.Vector(x=4, y=5)
        dist = p1.xyDist(p2)
        self.assertAlmostEqual(dist, 5.0)

        # test between quadrants
        p2 = coord.Vector(x=-2, y=-3)
        dist = p1.xyDist(p2)
        self.assertAlmostEqual(dist, 5.0)

    def test_vectorSubtract(self):
        # test quadrant 1
        p1 = coord.Vector(x=5, y=5)
        p2 = coord.Vector(x=2, y=3)
        sub = p1.vectorSubtract(p2)
        self.assertAlmostEqual(sub.x, 3)
        self.assertAlmostEqual(sub.y, 2)

        # test between quadrants
        p2 = coord.Vector(x=-2, y=-3)
        sub = p1.vectorSubtract(p2)
        self.assertAlmostEqual(sub.x, 7)
        self.assertAlmostEqual(sub.y, 8)

    def test_dot(self):
        v1 = coord.Vector(x=3, y=2)
        v2 = coord.Vector(x=-2, y=5)
        dot_prod = v1.dot(v2)
        self.assertAlmostEqual(dot_prod, 4)

    def test_magnitude(self):
        v = coord.Vector(x=3, y=4)
        self.assertAlmostEqual(v.magnitude(), 5)

        v = coord.Vector(x=-3, y=4)
        self.assertAlmostEqual(v.magnitude(), 5)

    def test_angle(self):
        v = coord.Vector(x=5, y=5)
        self.assertAlmostEqual(v.angle(), 45)

        v = coord.Vector(x=3, y=4)
        self.assertAlmostEqual(v.angle(), 53.13, 2)

        v = coord.Vector(x=-5, y=5)
        self.assertAlmostEqual(v.angle(), 135)

        v = coord.Vector(x=-5, y=-5)
        self.assertAlmostEqual(v.angle(), 225)

        v = coord.Vector(x=5, y=-5)
        self.assertAlmostEqual(v.angle(), 315)

    def test_toUnitVector(self):
        # test quadrant 1
        v = coord.Vector(x=5, y=5)
        unit_vec = v.toUnitVector()
        self.assertAlmostEqual(v.angle(), unit_vec.angle())
        self.assertAlmostEqual(unit_vec.magnitude(), 1)

        # test quadrant 3
        v = coord.Vector(x=-5, y=-5)
        unit_vec = v.toUnitVector()
        self.assertAlmostEqual(v.angle(), unit_vec.angle())
        self.assertAlmostEqual(unit_vec.magnitude(), 1)

    def test_inverse(self):
        # test quadrant 1
        v = coord.Vector(x=2, y=3)
        inv = v.inverse()
        self.assertAlmostEqual(inv.x, -v.x)
        self.assertAlmostEqual(inv.y, -v.y)

        # test quadrant 4
        v = coord.Vector(x=2, y=-3)
        inv = v.inverse()
        self.assertAlmostEqual(inv.x, -v.x)
        self.assertAlmostEqual(inv.y, -v.y)

    def test_midpoint(self):
        # test quadrant 1
        p1 = coord.Vector(x=1, y=1)
        p2 = coord.Vector(x=4, y=5)
        mid = p1.midpoint(p2)
        self.assertAlmostEqual(mid.x, 2.5)
        self.assertAlmostEqual(mid.y, 3)

        # test between quadrants
        p2 = coord.Vector(x=-4, y=-5)
        mid = p1.midpoint(p2)
        self.assertAlmostEqual(mid.x, -1.5)
        self.assertAlmostEqual(mid.y, -2)

    def test_zeroVector(self):
        z = coord.Vector.zeroVector()
        self.assertAlmostEqual(z.x, 0)
        self.assertAlmostEqual(z.y, 0)
        self.assertAlmostEqual(z.magnitude(), 0)
        self.assertAlmostEqual(z.angle(), 0)

    def test_degToRad(self):
        self.assertAlmostEqual(coord.degToRad(0), 0)
        self.assertAlmostEqual(coord.degToRad(180), 3.14159, 5)

    def test_radToDeg(self):
        self.assertAlmostEqual(coord.radToDeg(0), 0)
        self.assertAlmostEqual(coord.radToDeg(3.14159), 180, 1)

    def test_rangeAngle(self):
        self.assertAlmostEqual(coord.rangeAngle(0), 0)
        self.assertAlmostEqual(coord.rangeAngle(-180), 180)
        self.assertAlmostEqual(coord.rangeAngle(-722), 358)
        self.assertAlmostEqual(coord.rangeAngle(720), 0)
        self.assertAlmostEqual(coord.rangeAngle(820), 100)


if __name__ == '__main__':
    unittest.main()
