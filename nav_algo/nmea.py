class NMEA:
    """A parser for National Marine Electronics Association (NMEA) sentences.

    The Parallax SIM33EAU GPS module transmits six different types of NMEA
    sentences: RMC: Recommended minimum data for GPS, VTG: Vector track and 
    speed over the ground, GGA: Fix information, GSA: Overall satellite data, 
    GSV: Detailed satellite data, and GLL: Latitude/Longitude data. This parser 
    can account for each of these types. More information about NMEA sentences
    can be found here: https://www.gpsinformation.org/dale/nmea.htm.

    The parsed information can be accessed through attributes of the NMEA
    object, which may be `None` if that attribute does not exist in the given
    sentence type.

    Args:
        sentence (str): An NMEA sentence to parse.
    
    Raises:
        TypeError: If the input sentence is not a string.
        ValueError: If the input is not a valid NMEA sentence.

    Attributes:
        utc (UTC): The UTC time at which the NMEA sentence was generated.
        status (bool): Whether the NMEA sentence contained valid data.
        latitude (float): The numeric latititude measurement.
        longitude (float): The numeric longitude measurement.

    """
    def __init__(self, sentence: str):
        """Initializes an NMEA obect using the string sentence, then parses it."""
        if not isinstance(sentence, str):
            raise TypeError('Input sentence must be a string.')

        self.sentence = sentence
        self.parse()

    def parse(self):
        """Parses the sentence associated with the NMEA object."""
        self.fields = self.sentence.split(',')

        if "RMC" in self.sentence:
            self.parseRMC()
        elif "VTG" in self.sentence:
            self.parseVTG()
        elif "GGA" in self.sentence:
            self.parseGGA()
        elif "GSA" in self.sentence:
            self.parseGSA()
        elif "GSV" in self.sentence:
            self.parseGSV()
        elif "GLL" in self.sentence:
            self.parseGLL()
        else:
            raise ValueError('Input is not a valid NMEA sentence.')

    def parseRMC(self):
        """Parses RMC sentences.

        RMC sentences look something like: 
        $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        and contain the UTC time, status (active (A) if the data is valid, void
        (V) otherwise), latitude, longitude, and some satellite data, which 
        we will ignore since we have no use for it. These values are stored
        as attributes of the NMEA object.
        """
        self.utc = self.UTC(self.fields[1])
        self.status = 'A' in self.fields[2]
        self.latDecimalDegrees(self.fields[3], self.fields[4])
        self.longDecimalDegrees(self.fields[5], self.fields[6])

    def parseVTG(self):
        """Parses VTG sentences.

        VTG sentences look something like: 
        $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
        and are pretty much useless to us, so this method does nothing except
        set status to `False`.
        """
        self.status = False

    def parseGGA(self):
        """Parses GGA sentences.

        GGA sentences look something like: 
        $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        and contain the UTC time, latitude, longitude, the fix status, and some 
        satellite data, which we will ignore since we have no use for it. These 
        values are stored as attributes of the NMEA object.
        """
        self.utc = self.UTC(self.fields[1])
        self.latDecimalDegrees(self.fields[2], self.fields[3])
        self.longDecimalDegrees(self.fields[4], self.fields[5])
        self.status = not '0' in self.fields[6]

    def parseGSA(self):
        """Parses GSA sentences.

        GSA sentences look something like: 
        $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
        and are pretty much useless to us, so this method does nothing except
        set status to `False`.
        """
        self.status = False

    def parseGSV(self):
        """Parses GSV sentences.

        GSV sentences look something like: 
        $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*7
        and are pretty much useless to us, so this method does nothing except
        set status to `False`.
        """
        self.status = False

    def parseGLL(self):
        """Parses GLL sentences.

        GLL sentences look something like: 
        $GPGLL,4916.45,N,12311.12,W,225444,A,*1D
        and contain the latitude, longitude, UTC time, status (active (A) if 
        the data is valid, void (V) otherwise), and some satellite data, which 
        we will ignore since we have no use for it. These values are stored
        as attributes of the NMEA object.
        """
        self.latDecimalDegrees(self.fields[1], self.fields[2])
        self.longDecimalDegrees(self.fields[3], self.fields[4])
        self.utc = self.UTC(self.fields[5])
        self.status = 'A' in self.fields[6]

    def latDecimalDegrees(self, nmea_format: str, dir: str):
        """Converts latitude from NMEA format to decimal degrees.

        NMEA format is DDMM.MMMM where DD is the degrees and MM.MMMM is the
        minutes. The decimal degreee value is DD + (MM.MMMM / 60). Values 
        in the northern hemisphere are positive and values in the southern
        hemisphere are negative.

        Args:
            nmea_format (str): The latitude in NMEA format.
            dir (str): N for north or S for south.
        """
        degs = float(nmea_format[0:2])
        mins = float(nmea_format[2:])
        self.latitude = degs + (mins / 60.0)
        if 'S' in dir:
            self.latitude = self.latitude * -1.0

    def longDecimalDegrees(self, nmea_format: str, dir: str):
        """Converts longitude from NMEA format to decimal degrees.

        NMEA format is DDDMM.MMMM where DDD is the degrees and MM.MMMM is the
        minutes. The decimal degreee value is DDD + (MM.MMMM / 60). Values 
        in the eastern hemisphere are positive and values in the western
        hemisphere are negative.

        Args:
            nmea_format (str): The longitude in NMEA format.
            dir (str): E for east or W for west.
        """
        degs = float(nmea_format[0:3])
        mins = float(nmea_format[3:])
        self.longitude = degs + (mins / 60.0)
        if 'W' in dir:
            self.longitude = self.longitude * -1.0

    class UTC():
        """A representation of Coordinated Universal Time (UTC).

        Args:
            time (str): the utc time to parse (a string of the form 'hhmmss').
    
        Raises:
            TypeError: If the input time is not a string.

        Attributes:
            hour (int): The hour of the UTC time (24 hour format).
            minute (int): The minute of the UTC time.
            second (int): The seconds of the UTC time.
        """
        def __init__(self, time: str):
            """ Initializes a UTC object with the given time. """
            if not isinstance(time, str):
                raise TypeError('Input time must be a string.')

            self.hour = int(time[0:2])
            self.minute = int(time[2:4])
            self.second = int(time[4:6])
