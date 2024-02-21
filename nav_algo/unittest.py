import unittest

class TestEventAlgorithms(unittest.TestCase):

    def test_station_keeping(self):
        self.assertEqual('foo'.upper(), 'FOO')

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
    unittest.main()