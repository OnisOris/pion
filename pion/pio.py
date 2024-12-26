from abc import ABC, abstractmethod
import numpy as np
from .annotation import *
import time

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
    @abstractmethod
    def _send_heartbeat(self):
        """
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном.
        :return: None
        """
        pass

class DroneBase(Pio):
    def __init__(self, 
                 name: str = "baseclass", 
                 mass: float = 0.3, 
                 dimension: int = 3,
                 position: Union[Array6, Array4, None] = None,
                 attitude: Union[Array6, Array4, None] = None,
                 count_of_checking_points: int = 20,
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
        self._heartbeat_timeout = 1
        self._heartbeat_send_time = time.time()
        # Используется для хранения последних count_of_checking_points данных в виде [x, y, z, yaw] для верификации достижения таргетной точки
        self.last_points = np.zeros((count_of_checking_points, 4))




    @property
    def position(self) -> np.ndarray:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return self._position

    @position.setter
    def position(self, position) -> None:
        """
        Сеттер для _position
        :return: None
        """
        self._position = position
        
    @property
    def yaw(self) -> np.ndarray:
        """
        Геттер вернет yaw
        :return: np.ndarray
        """
        return self.attitude[2]


    @property
    def attitude(self) -> np.ndarray:
        """
        Функция вернет ndarray (6,) с координатами roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        :return: np.ndarray
        """
        return self._attitude

    @attitude.setter
    def attitude(self, attitude) -> None:
        """
        Сеттер для _attitude
        :return: None
        """
        self._attitude = attitude
    def heartbeat(self) -> None:
        """
        Функция проверки heartbeat дрона
        :return: None
        """
        if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
            self._send_heartbeat()


    def _send_heartbeat(self):
        """
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном.
        :return: None
        """
        pass


