from abc import ABC, abstractmethod

class Pio(ABC):
    @abstractmethod
    def arm(self):
        pass
    @abstractmethod
    def disarm(self):
        pass
    @abstractmethod
    def takeoff(self):
        pass
    @abstractmethod
    def land(self):
        pass
    @abstractmethod
    def goto(self):
        pass
    @abstractmethod
    def set_v(self):
        pass
    @abstractmethod
    def stop(self):
        pass

