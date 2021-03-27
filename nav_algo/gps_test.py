from sensors import *
import time
sd = sensorData()

while True:
    time.sleep(2)
    sd.readGPS()
    print('lat: {}, long: {}'.format(sd.latitude, sd.longitude))