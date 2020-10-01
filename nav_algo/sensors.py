from abc import ABC, abstractmethod


class sensorData(ABC):
    @abstractmethod
    def readAll(self):
        pass
