import numpy as np


class CoordinateSystem:
    """A representation of a latitude/longitude coordinate system.

    An object of type CoordinateSystem can be used to initialize a vector inside 
    of a coordinate system with origin at LAT_OFFSET and LONG_OFFSET.

    Args:
        latitude (float): The latitude of the origin of the system.
        longitude (float): The longitude of the origin of the system.

    Attributes:
        LAT_OFFSET (float): The latitude of the origin of the system.
        LONG_OFFSET (float): The longitude of the origin of the system.
        EARTH_RADIUS (float): The radius of the Earth (in meters).

    """
    def __init__(self, latititude: float, longitude: float):
        self.LAT_OFFSET = latititude
        self.LONG_OFFSET = longitude
        self.EARTH_RADIUS = 6371000.0


class Vector:
    """A representation of 2D vectors with a coordinate system.

    To construct a position vector, pass in a CoordinateSystem and a latitude
    and longitude. To specify only the x and y components, pass in x and y.
    Alternatively, to construct a unit vector with a given angle, 
    pass in an angle.

    Args:
        coord_sys (CoordinateSystem): (Optional) The coordinate system in which the vector lies.
        latitude (float): (Optional) The latitude of the position.
        longitude (float): (Optional) The longitude of the position.
        angle (float): (Optional) The angle (in degrees) used to construct a unit vector.
        x (float): (Optional) The x component of the vector.
        y (float): (Optional) The y component of the vector.

    Attributes:
        x (float): The x component of the vector.
        y (float): The y component of the vector.
        coordinate_system (CoordinateSystem): (Optional) The coordinate system in which the vector lies.
        latitude (float): (Optional) The latitude of the position.
        longitude (float): (Optional) The longitude of the position.

    """
    def __init__(self,
                 coord_sys=None,
                 latitude=None,
                 longitude=None,
                 angle=None,
                 x=None,
                 y=None):
        if (coord_sys is not None) and (latitude
                                        is not None) and (longitude
                                                          is not None):
            self.coordinate_system = coord_sys
            self.latitude = latitude
            self.longitude = longitude

            shifted_long = longitude - coord_sys.LONG_OFFSET
            shifted_lat = latitude - coord_sys.LAT_OFFSET
            deg_to_rad = np.pi / 180.0

            self.x = coord_sys.EARTH_RADIUS * np.cos(
                coord_sys.LAT_OFFSET * deg_to_rad) * deg_to_rad * shifted_long
            self.y = coord_sys.EARTH_RADIUS * deg_to_rad * shifted_lat

        elif angle is not None:
            self.x = np.cos(degToRad(angle))
            self.y = np.sin(degToRad(angle))

        elif (x is not None) and (y is not None):
            self.x = x
            self.y = y

    def xyDist(self, other):
        """Calculates the distance between two positions.

        Args:
            other (Vector): The second position in the distance calculation.

        Returns:
            float: The distance between 'self' and 'other'.

        """
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def vectorSubtract(self, other):
        """Calculates the vector difference between two vectors.

        Args:
            other (Vector): The vector to subtract from 'self'.

        Returns:
            Vector: The vector difference between 'self' and 'other'.

        """
        return Vector(x=self.x - other.x, y=self.y - other.y)

    def dot(self, other):
        """Calculates the dot product of two vectors.

        Args:
            other (Vector): The vector to dot with 'self'.

        Returns:
            float: The dot product of 'self' and 'other'.

        """
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        """Calculates the magnitude of a vector.

        Returns:
            float: The magnitude of the vector.

        """
        return np.sqrt(self.x**2 + self.y**2)

    def angle(self):
        """Calculates the anglular distance between the vector and the x-axis.

        The returned angle is between 0 and 360 degrees.

        Returns:
            float: The angle between the vector and North.

        """
        return rangeAngle(radToDeg(np.arctan2(self.y, self.x)))

    def scale(self, scaleFactor):
        return Vector(x=self.x * scaleFactor, y=self.y * scaleFactor)

    def toUnitVector(self):
        """Converts a vector to a unit vector.

        Returns:
            Vector: A unit vector representation of the input vector.

        """
        mag = self.magnitude()
        return Vector(x=self.x / mag, y=self.y / mag)

    def inverse(self):
        """Constructs the inverse of a vector.

        Returns:
            Vector: The inverse of the original vector.
        
        """
        return Vector(x=-1.0 * self.x, y=-1.0 * self.y)

    def midpoint(self, other):
        """Calculates the midpoint between two positions.

        Args:
            other (Vector): The second position vector.

        Returns:
            Vector: The position vector midpoint between 'self' and 'other'.

        """
        dx = (self.x - other.x) / 2.0
        dy = (self.y - other.y) / 2.0

        return Vector(x=self.x - dx, y=self.y - dy)

    def angleBetween(self, other):
        """Calculates the angle between two vectors

        Args:
            self: vector
            other: vector
        
        Returns:
            float: An angle within the range of 0 to 360.
        
        """
        top = self.dot(other)
        bot = self.magnitude() * other.magnitude()
        return radToDeg(np.arccos(top / bot))

    @staticmethod
    def zeroVector():
        """Constructs a zero vector.

        Returns:
            Vector: A vector with zero magnitude.
        
        """
        return Vector(x=0.0, y=0.0)
    

    @staticmethod
    def convertXYToLatLong(coord_sys, x, y):
        radEarth = coord_sys.EARTH_RADIUS
        latOff = coord_sys.LAT_OFFSET
        longOff = coord_sys.LONG_OFF
        radDeg = 180.0 / np.pi

        long = (x * radDeg) / (radEarth * np.cos(np.deg2rad(latOff))) + longOff
        lat = (y * radDeg) / radEarth + latOff

        return Vector(coord_sys=coord_sys, latitude=lat, longitude=long)


def degToRad(angle):
    """Converts from degrees to radians.

    Args:
        angle (float): An angle in degrees.
    
    Returns:
        float: The angle in radians.
    
    """
    return angle * (np.pi / 180.0)


def radToDeg(angle):
    """Converts from radians to degrees.

    Args:
        angle (float): An angle in radians.
    
    Returns:
        float: The angle in degrees.
    
    """
    return angle * (180.0 / np.pi)


def rangeAngle(angle):
    """Puts an angle with the range of 0 to 360.

    Args:
        angle (float): An angle in degrees.
    
    Returns:
        float: An angle within the range of 0 to 360.
    
    """
    return angle % 360
