from abc import ABC, abstractmethod
import numpy as np
from .annotation import *

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

class DroneBase(Pio):
    def __init__(self, 
                 name: str = "baseclass", 
                 mass: float = 0.3, 
                 dimension: int = 3,
                 position: Union[Array6, Array4, None] = None,
                 attitude: Union[Array6, Array4, None] = None,
                 logger: bool = False):
        self.name = name
        self.mass = mass
        self.dimension = dimension
        self.logger = logger
        if position is None:
            position = np.zeros(self.dimension * 2)
        else:
            if position.shape not in [(4,), (6,)]:
                raise ValueError(f"Размерность вектора position должна быть равна 4 или 6")
        if attitude is None:
            attitude = np.zeros(6)
        # Вектор, подобный LOCAL_POSITION_NED из mavlink
        self._position = position
        # Вектор, подобный ATTITUDE из mavlink
        self._attitude = attitude
        self.t_speed = np.zeros(self.dimension+1)  # [vx, vy, vz, yaw_rate]
        self.position_pid_matrix = np.array([
            [0.5] * self.dimension,
            [0.] * self.dimension,
            [2.] * self.dimension
        ], dtype=np.float64)

