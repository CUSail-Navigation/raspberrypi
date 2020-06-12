import unittest
import nmea


class TestNmeaMethods(unittest.TestCase):
    def test_init(self):
        # make sure it returns the correct type
        n = nmea.NMEA(
            '$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47'
        )
        self.assertIsInstance(n, nmea.NMEA)

        # make sure it won't try to parse something that isn't a string
        self.assertRaises(TypeError, lambda: nmea.NMEA(12))

        # make sure it won't try to parse something that isn't an NMEA sentence
        self.assertRaises(ValueError, lambda: nmea.NMEA("test"))

    def test_RMC(self):
        # make sure a valid sentence gets parsed correctly
        n = nmea.NMEA(
            '$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A'
        )
        self.assertEqual(n.utc.hour, 12)
        self.assertEqual(n.utc.minute, 35)
        self.assertEqual(n.utc.second, 19)
        self.assertTrue(n.status)
        self.assertAlmostEqual(n.latitude, 4807.038)
        self.assertTrue(n.north)
        self.assertAlmostEqual(n.longitude, 01131.000)
        self.assertFalse(n.west)

        # make sure an invalid sentence has False status
        n = nmea.NMEA(
            '$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A'
        )
        self.assertFalse(n.status)

    def test_VTG(self):
        # make sure status is false
        n = nmea.NMEA('$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48')
        self.assertFalse(n.status)

    def test_GGA(self):
        # make sure a valid sentence gets parsed correctly
        n = nmea.NMEA(
            '$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*47'
        )
        self.assertEqual(n.utc.hour, 12)
        self.assertEqual(n.utc.minute, 35)
        self.assertEqual(n.utc.second, 19)
        self.assertTrue(n.status)
        self.assertAlmostEqual(n.latitude, 4807.038)
        self.assertFalse(n.north)
        self.assertAlmostEqual(n.longitude, 01131.000)
        self.assertTrue(n.west)

        # make sure an invalid sentence has False status
        n = nmea.NMEA(
            '$GPGGA,123519,4807.038,S,01131.000,W,0,08,0.9,545.4,M,46.9,M,,*47'
        )
        self.assertFalse(n.status)

    def test_GSA(self):
        # make sure status is false
        n = nmea.NMEA('$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39')
        self.assertFalse(n.status)

    def test_GSV(self):
        # make sure status is false
        n = nmea.NMEA(
            '$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75'
        )
        self.assertFalse(n.status)

    def test_GLL(self):
        # make sure a valid sentence gets parsed correctly
        n = nmea.NMEA('$GPGLL,4916.45,N,12311.12,W,225444,A,*1D')
        self.assertEqual(n.utc.hour, 22)
        self.assertEqual(n.utc.minute, 54)
        self.assertEqual(n.utc.second, 44)
        self.assertTrue(n.status)
        self.assertAlmostEqual(n.latitude, 4916.45)
        self.assertTrue(n.north)
        self.assertAlmostEqual(n.longitude, 12311.12)
        self.assertTrue(n.west)

        # make sure an invalid sentence has False status
        n = nmea.NMEA('$GPGLL,4916.45,N,12311.12,W,225444,V,*1D')
        self.assertFalse(n.status)


if __name__ == '__main__':
    unittest.main()