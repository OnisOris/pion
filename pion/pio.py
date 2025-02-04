from abc import ABC, abstractmethod
import numpy as np
from .annotation import *
import time
from collections import deque
from rich.console import Console
from rich.table import Table


class Pio(ABC):
    """
    Абстрактный класс Pio с важными методами для дрона
    """
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


class DroneBase(Pio, ABC):
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 name: str = "baseclass",
                 mass: float = 0.3,
                 dimension: int = 3,
                 position: Union[Array6, Array4, None] = None,
                 attitude: Union[Array6, None] = None,
                 count_of_checking_points: int = 20,
                 logger: bool = False,
                 checking_components: bool = True,
                 accuracy: float = 5e-2,
                 dt: float = 0.1,
                 max_speed: float = 2.):
        # Время создания экземпляра
        self.t0 = time.time()
        self.ip = ip
        self.mavlink_port = mavlink_port
        self.name = name
        self.mass = mass
        self.dimension = dimension
        self.dt = dt
        self.logger = logger
        self.logs = {}
        self.checking_components = checking_components
        self.dimension = dimension
        self._pid_position_controller = None
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
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
        self.t_speed = np.zeros(4)  # [vx, vy, vz, yaw_rate]
        self.position_pid_matrix = np.array([
            [0.5] * self.dimension,
            [0.] * self.dimension,
            [2.] * self.dimension
        ], dtype=np.float64)
        self._heartbeat_timeout = 1
        self._heartbeat_send_time = time.time()
        # Используется для хранения последних count_of_checking_points данных в виде [x, y, z, yaw] для верификации достижения таргетной точки
        self.last_points = np.zeros((count_of_checking_points, 4))
        # Напряжение батареи дрона (Вольты)
        self.battery_voltage = None
        # Флаг для запуска и остановки сохранения координат
        self.check_attitude_flag = False
        # Информация, включающая
        # x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t
        # которая складывается в матрицу (n, 17/14), где n - число точек в траектории
        # если размерность 2, то z составляющая убирается из траектории и размерность вектора равна 14, а не 17
        self.trajectory = np.zeros((2, self._position.shape[0] + self._attitude.shape[0] + self.t_speed.shape[0] + 1))
        self.accuracy = accuracy
        self.point_reached = False
        self.max_speed = max_speed
        self._console = Console()


    @property
    def position(self) -> Union[Array6, Array4]:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return self._position

    @position.setter
    def position(self, position: Union[Array6, Array4]) -> None:
        """
        Сеттер для _position
        :return: None
        """
        self._position = position
    @property
    def xyz(self) -> Union[Array3, Array2]:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return self.position[0:self.dimension]

    @xyz.setter
    def xyz(self, position: Union[Array3, Array2]) -> None:
        """
        Сеттер для _position
        :return: None
        """
        self.position[0:self.dimension] = position
    @property
    def yaw(self) -> np.ndarray:
        """
        Геттер вернет yaw
        :return: np.ndarray
        """
        return self.attitude[2]

    @property
    def attitude(self) -> Union[Array6, Array4]:
        """
        Функция вернет ndarray (6,) с координатами roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        :return: np.ndarray
        """
        return self._attitude

    @attitude.setter
    def attitude(self, attitude: Union[Array6, Array4]) -> None:
        """
        Сеттер для _attitude
        :return: None
        """
        self._attitude = attitude
    # Реализация обязательных методов абстрактного класса Pio
    def arm(self) -> None:
        """
        Включает двигатели
        :return: None
        """
        if self.logger:
            self.logs.update({"Status": f"{self.name} is armed \n"})

    def disarm(self) -> None:
        """
        Отключает двигатели
        :return: None
        """
        if self.logger:
            self.logs.update({"Status": f"{self.name} is disarmed \n"})

    def takeoff(self) -> None:
        """
        Взлет дрона
        :return: None
        """
        if self.logger:
            self.logs.update({"Status": f"{self.name} is take off \n"})

    def land(self) -> None:
        """
        Посадка дрона
        :return: None
        """
        if self.logger:
            self.logs.update({"Status": f"{self.name} is landing \n"})



    def heartbeat(self) -> None:
        """
        Функция проверки heartbeat дрона
        :return: None
        """
        if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
            self._send_heartbeat()

    def _send_heartbeat(self) -> None:
        """
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном.
        :return: None
        """
        pass

    def goto_yaw(self,
                 yaw: Union[float, int] = 0,
                 accuracy: Union[float, int] = 0.087) -> None:
        """
        Берет целевую координату по yaw и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.
        Для использования необходимо включить цикл v_while для посылки вектора скорости дрону.
        Максимальная скорость обрезается np.clip по полю self.max_speed.
        :param yaw:  координата по yaw (радианы)
        :type: Union[float, int]
        :param accuracy: Погрешность целевой точки
        :type: Union[float, int]
        :return: None
        """
        return None

    def led_control(self,
                    led_id=255,
                    r=0,
                    g=0,
                    b=0) -> None:
        """
        Управление светодиодами на дроне.

        :param led_id: Идентификатор светодиода, который нужно управлять. Допустимые значения: 0, 1, 2, 3, 255.
        255 — для управления всеми светодиодами одновременно.
        :type led_id: int
        :param r: Значение интенсивности красного канала (от 0 до 255).
        :type r: int
        :param g: Значение интенсивности зеленого канала (от 0 до 255).
        :type g: int
        :param b: Значение интенсивности синего канала (от 0 до 255).
        :type b: int
        :raises ValueError: Если переданы недопустимые значения для параметра led_id или для значений r, g, b.
        :return: None
            """
        pass

    def save_data(self,
                  file_name: str = 'data.npy',
                  path: str = '') -> None:
        """
        Функция для сохранения траектории в файл
        columns=['x', 'y', 'z', 'vx', 'vy', 'yaw', 'pitch', 'roll','Vyaw', 'Vpitch', 'Vroll', 'vxc', 'vyc', 'vzc', 'v_yaw_c', 't']
        :param file_name: название файла
        :type: str
        :param path: путь сохранения
        :type: str
        :return: None
        """
        self.stop()
        np.save(f'{path}{file_name}', self.trajectory[2:])

    def check_battery(self) -> None:
        """
        Проверяет статус батареи
        :return: None
        """
        voltage = self.battery_voltage
        if voltage is not None:
            if voltage < 6.9:
                print(f">>>>>>>>>>>>>>>>>>Аккумулятор разряжен<<<<<<<<<<<<<<<<<<<<<<\nvoltage = {voltage}")
            else:
                print(f"voltage = {voltage}")
        else:
            print("Сообщение о статусе батареи еще не пришло")

    def reboot_board(self) -> None:
        """
        Функция для перезагрузки дрона
        :return: None
        """
        pass

    def attitude_write(self) -> None:
        """
        Функция для записи траектории в numpy массив. Записывается только уникальная координата
        :return:
        """
        t = time.time() - self.t0
        stack = np.hstack([self.position, self.attitude, self.t_speed, [t]])
        if not np.all(np.equal(stack[:-1], self.trajectory[-2, :-1])):
            self.trajectory = np.vstack([self.trajectory, stack])

    def set_v(self) -> None:
        """
        Создает поток, который вызывает функцию v_while() для параллельной отправки вектора скорости
        :return: None
        """
        pass

    def position_controller(self, position_xyz: Union[Array3, Array2], dt: float):
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(position_xyz, dtype=np.float64),
                current_position=self.xyz,
                dt=dt),
            -self.max_speed,
            self.max_speed)
        self.t_speed = np.hstack([signal, np.array([0]*(4-self.dimension))])

    def print_information(self) -> None:
        """

        """
        self._console.clear()
        self.logs.update({"xyz":  f"{np.round(self.position[0:self.dimension], 3)} \n",
                         f"speed": f"{np.round(self.position[self.dimension:self.dimension * 2], 3)} \n",
                         f"t_speed": f"{np.round(self.t_speed, 3)} \n"})
        self.print_latest_logs(self.logs, 5, "Таблица с сообщениями")
  
    def print_latest_logs(self, log_dict: dict, n: int = 5, name: str = "Название"):
        # Берем последние n записей из словаря
        latest_logs = deque(log_dict.items(), maxlen=n)

        # Создаем таблицу для красивого вывода
        table = Table(title=f"{name}")

        table.add_column("ID", style="cyan", justify="right")
        table.add_column("Сообщение", style="green")

        for log_id, message in latest_logs:
            table.add_row(str(log_id), message)

        self._console.print(table)
