import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from typing import Optional, Union

import numpy as np
from rich.live import Live
from rich.table import Table

from pion.cython_pid import PIDController
from pionfunc.annotation import (
    Array2,
    Array3,
    Array4,
    Array6,
    Array17,
)


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

    def stop_moving(self) -> None:
        """
        Останавливает все движение

        :return: None
        """
        print("STOP_MOVING")

    @abstractmethod
    def _send_heartbeat(self):
        """
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном.

        :return: None
        """
        pass


class DroneBase(Pio, ABC):
    """
    Абстрактный класс с частичной реализацией методов, служит для сокращения кода в дочерних классах
    """

    def __init__(
        self,
        ip: str = "10.1.100.114",
        mavlink_port: int = 5656,
        name: str = "baseclass",
        mass: float = 0.3,
        position: Optional[Array6] = None,
        attitude: Optional[Array6] = None,
        count_of_checking_points: int = 20,
        logger: bool = False,
        checking_components: bool = True,
        accuracy: float = 5e-2,
        dt: float = 0.1,
        max_speed: float = 2.0,
        max_acceleration: float = 10,
    ):
        """
        Абстрактный класс с частичной реализацией методов, служит для сокращения кода в дочерних классах

        :param ip: IP-адрес для подключения к дрону.
        :type ip: str

        :param mavlink_port: Порт для MAVLink соединения.
        :type mavlink_port: int

        :param count_of_checking_points: Количество последних точек, используемых для проверки достижения цели.
        :type count_of_checking_points: int

        :param name: Название экземпляра
        :type name: str

        :param mass: Масса дрона
        :type mass: float

        :param position: Начальное состояние дрона вида [x, y, z, vx, vy, vz]
        :type position: Array6

        :param attitude: Начальное состояние дрона вида [roll, pitch, yaw, v_roll, v_pitch, v_yaw]
        :type attitude: Optional[Array6]

        :param dt: Период приема всех сообщений с дрона или шаг времени в симуляции в Spion
        :type dt: float

        :param logger: Включить логирование
        :type logger: bool

        :param checking_components: Параметр для проверки номеров компонентов. Отключается для в сторонних симуляторах
         во избежание ошибок.
        :type checking_components: bool

        :param accuracy: Максимальное отклонение от целевой позиции для функции goto_from_outside
        :type accuracy: float

        :param max_speed: Максимальная скорость дрона в режиме управления по скорости
        :type max_speed: float
        """

        self.max_yaw_rate: float = 1.0
        self.t0: float = time.time()
        self.ip: str = ip
        self.mavlink_port: int = mavlink_port
        self.name: str = name
        self.mass: float = mass
        self.dt: float = dt
        self.logger: bool = logger
        self.logs: dict = {}
        self.checking_components: bool = checking_components
        self._pid_position_controller: PIDController = None
        if position is None:
            position = np.zeros(6)
        else:
            if position.shape != (6,):
                raise ValueError(
                    "Размерность вектора position должна быть равна 6"
                )
        if attitude is None:
            attitude = np.zeros(6)
        # Вектор, подобный LOCAL_POSITION_NED из mavlink
        self._position: Array6 = position
        # Вектор, подобный ATTITUDE из mavlink
        self._attitude: Array6 = attitude
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
        self.t_speed = np.zeros(4)  # [vx, vy, vz, yaw_rate]
        self.position_pid_matrix: np.ndarray = np.array(
            [
                [0.5] * 3,
                [0.0] * 3,
                [2.0] * 3,
            ],
            dtype=np.float64,
        )
        self._heartbeat_timeout: float = 1.0
        self._heartbeat_send_time: float = time.time()
        # Используется для хранения последних count_of_checking_points данных в виде [x, y, z, yaw] для верификации достижения таргетной точки
        self.last_points: np.ndarray = np.zeros((count_of_checking_points, 4))
        # Напряжение батареи дрона (Вольты)
        self.battery_voltage: Optional[float] = None
        # Флаг для запуска и остановки сохранения координат
        self.check_attitude_flag: bool = False
        self.description_traj_fromat = """trajectory - информация, включающая
        x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t
        которая складывается в матрицу (n, 17), где n - число точек в траектории
        """
        self.trajectory: Array17 = np.zeros(
            (
                2,
                self._position.shape[0]
                + self._attitude.shape[0]
                + self.t_speed.shape[0]
                + 1,
            )
        )
        self.accuracy: float = accuracy
        self.point_reached: bool = False
        self.max_speed: float = max_speed
        self.max_acceleration: float = max_acceleration
        self._handler_lock: threading.Lock = threading.Lock()
        self._speed_control_lock: threading.Lock = threading.Lock()
        self._live: Optional[Live] = None
        self.target_point: np.ndarray = np.array([0, 0, 2, 0])
        # Флаг включения режима трекинга за точкой в отдельном потока: дрон будет следовать за точкой из поля target_point
        self.tracking: bool = False
        # Список потоков
        self.threads: list = []

    @property
    def position(self) -> Union[Array6, Array4]:
        """
        Метод вернет ndarray (6,) с координатами x, y, z, vx, vy, vz

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
        Метод вернет ndarray (6,) с координатами x, y, z, vx, vy, vz

        :return: np.ndarray
        """
        return self.position[0:3]

    @xyz.setter
    def xyz(self, position: Union[Array3, Array2]) -> None:
        """
        Сеттер для _position

        :return: None
        """
        self.position[0:3] = position

    @property
    def yaw(self) -> float:
        """
        Геттер вернет yaw

        :return: float
        """
        return self.attitude[2]

    @property
    def attitude(self) -> Union[Array6, Array4]:
        """
        Метод вернет ndarray (6,) с координатами roll, pitch, yaw, rollspeed, pitchspeed, yawspeed

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

    def stakeoff(self, hight: float = 1.5) -> None:
        """
        Умный взлет, блокрует основной поток, пока не взлетит

        :param hight: Высота взлета
        :type hight: float
        :return: None
        """
        self.takeoff()

    def sland(self, hight_of_disarm: float = 0.3) -> None:
        """
        Умная посадка дрона, выключает двигатели автоматически, если высота меньше hight_of_disarm

        :param hight_of_disarm: высота выключения двигателей
        :type hight_of_disarm: float
        :return: None
        """
        pass

    def land(self) -> None:
        """
        Посадка дрона

        :return: None
        """
        if self.logger:
            self.logs.update({"Status": f"{self.name} is landing \n"})

    def heartbeat(self) -> None:
        """
        Метод проверки heartbeat дрона

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

    def goto_yaw(
        self, yaw: Union[float, int] = 0, accuracy: Union[float, int] = 0.087
    ) -> None:
        """
        Берет целевую координату по yaw и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.

        Для использования необходимо включить цикл v_while для посылки вектора скорости дрону.
        Максимальная скорость обрезается np.clip по полю self.max_speed.

        :param yaw:  координата по yaw (радианы)
        :param accuracy: Погрешность целевой точки

        :return: None
        """
        return None

    def led_control(self, led_id=255, r=0, g=0, b=0) -> None:
        """
        Управление светодиодами на дроне.

        :param led_id: Идентификатор светодиода, который нужно управлять. Допустимые значения: 0, 1, 2, 3, 255. 255 — для управления всеми светодиодами одновременно.
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

    def save_data(self, file_name: str = "data.npy", path: str = "") -> None:
        """
        Метод для сохранения траектории в файл.

        columns=['x', 'y', 'z', 'vx', 'vy', 'yaw', 'pitch', 'roll','Vyaw', 'Vpitch', 'Vroll', 'vxc', 'vyc', 'vzc', 'v_yaw_c', 't']

        :param file_name: название файла
        :param path: путь сохранения
        :return: None
        """
        self.stop()
        np.save(f"{path}{file_name}", self.trajectory[2:])

    def check_battery(self) -> None:
        """
        Проверяет статус батареи

        :return: None
        """
        voltage = self.battery_voltage
        if voltage is not None:
            if voltage < 6.9:
                print(
                    f">>>>>>>>>>>>>>>>>>Аккумулятор разряжен<<<<<<<<<<<<<<<<<<<<<<\nvoltage = {voltage}"
                )
            else:
                print(f"voltage = {voltage}")
        else:
            print("Сообщение о статусе батареи еще не пришло")

    def reboot_board(self) -> None:
        """
        Метод для перезагрузки дрона

        :return: None
        """
        pass

    def attitude_write(self) -> None:
        """
        Метод для записи траектории в numpy массив. Записывается только уникальная координата

        :return: None
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

    def position_controller(
        self, position_xyz: Union[Array3, Array2], dt: float
    ) -> None:
        """
        Фнкция формирования управляющего сигнала в сторону position_xyz

        :param position_xyz: Целевая координата
        :type position_xyz: Union[Array3, Array2]

        :param dt: шаг времени расчета
        :type dt: float

        :return: None
        :rtype: None
        """
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(position_xyz, dtype=np.float64),
                current_position=self.xyz,
                dt=dt,
            ),
            -self.max_speed,
            self.max_speed,
        )
        self.t_speed = np.hstack([signal, [0]])

    def print_information(self) -> None:
        """
        Метод обновляет словарь с логами self.logs

        :return: None
        """
        self._cleanup_threads()
        self.logs.update(
            {
                "name:": f"{self.name}",
                "xyz": f"{np.round(self.position[0:3], 3)} \n",
                "speed": f"{np.round(self.position[3:6], 3)} \n",
                "yaw": f"{self.yaw}",
                "t_speed": f"{np.round(self.t_speed, 3)} \n",
                "target_point": f"{self.target_point}",
                "battery voltage": f"{self.battery_voltage} \n",
                "threads": f"{self.threads} \n",
            }
        )
        self.print_latest_logs(self.logs, 8, "Таблица с сообщениями")

    def print_latest_logs(
        self, log_dict: dict, n: int = 5, name: str = "Название"
    ) -> None:
        """
        Метод обновляет результаты в таблице логов

        :param log_dict: Словарь с логами заполнения таблицы
        :type log_dict: dict

        :param n: Количество логов из словаря, которые попадут в таблицу
        :type n: int

        :param name: Заголовок таблицы
        :type name: str

        :return: None
        """
        latest_logs = deque(log_dict.items(), maxlen=n)
        # Создаем новую таблицу при каждом обновлении
        table = Table(title=name)
        table.add_column("ID", style="cyan", justify="right")
        table.add_column("Сообщение", style="green")

        for log_id, message in latest_logs:
            table.add_row(
                str(log_id), message.strip()
            )  # Убираем лишние переносы

        # Инициализируем Live один раз
        if self._live is None:
            self._live = Live(table, refresh_per_second=20, transient=False)
            self._live.start()
        else:
            self._live.update(table)

    def _cleanup_threads(self):
        """Очищает список потоков от завершенных."""
        self.threads = [t for t in self.threads if t.is_alive()]

    def trajectory_tracking(self, path_to_traj_file: str = "./data.npy"):
        pass

    def send_rc_channels(
            self,
            channel_1: int = 1500,
            channel_2: int = 1500,
            channel_3: int = 1500,
            channel_4: int = 1500,
    ) -> None:
        """
        Управление по rc-каналам
        """
        pass

