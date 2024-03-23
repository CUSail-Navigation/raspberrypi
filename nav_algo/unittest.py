import unittest
from nav_algo.coordinates import Vector
from nav_algo.event_tests.station_keeping_test import StationKeepingTests

class TestEventAlgorithms(unittest.TestCase):

    def test_station_keeping(self):
        test = StationKeepingTests()
        self.assertTrue(test.entryStationKeepingTest(Vector(x=0.0, y=50.0), 0, 25))
        self.assertTrue(test.entryStationKeepingTest(Vector(x=100.0, y=50.0), 80, 20))

    def test_endurance(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_search(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)

    def test_precision_navigation(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())
    
    def test_collision_avoidance(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())
    

if __name__ == '__main__':
    # unittest.main()
    test = TestEventAlgorithms()
    test.test_station_keeping()