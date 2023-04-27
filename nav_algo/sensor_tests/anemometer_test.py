import nav_algo.low_level.SailSensors as SailSensors

sailAngleBoat  = -90
boat_direction = 0
anemomSMA=[]
anemometer = SailSensors.SailAnemometer(0)
def readWindDirection():
        rawData = anemometer.readAnemometerVoltage()
        """print(rawData)"""
        rawWind = rawData
        rawAngle = 360 - rawData * 360 / 1700

        windWrtN = (rawAngle + sailAngleBoat) % 360
        windWrtN = (windWrtN + boat_direction) % 360
        wind_direction = _addAverage(windWrtN)
        print(wind_direction)
        return
    
    
def _addAverage(newValue):
        """Helper function that manages the SMA of the anemometer, this keeps the list at size =11 and returns the
        average of the list of ints. This function assumes that anemometer readings are taken semi-frequently
        parameter: newValue - int denoting number to be added to the """
        anemomSMA.append(newValue / 2)
        if (len(anemomSMA) > 1):
            for i in range(len(anemomSMA)-1):
                anemomSMA[i] = anemomSMA[i] / 2
        if (len(anemomSMA) > 10):
            anemomSMA.pop(0)
        sum = 0
        for n in anemomSMA:
            sum = sum + n
        return sum

while (1):
    readWindDirection()


