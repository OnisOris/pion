from .simulator import Simulator, Point
from .pio import DroneBase
from .functions import vector_reached, update_array
from pion.cython_pid import PIDController  # Cython-версия PIDController
from .annotation import *
import numpy as np
import time
import threading


class Spion(Simulator, DroneBase):
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 connection_method: str = 'udpout',
                 combine_system: int = 0,
                 count_of_checking_points: int = 20,
                 name: str = 'simulator',
                 mass: float = 0.3,
                 position: Union[Array6, Array4, None] = None,
                 attitude: Union[Array6, Array4, None] = None,
                 dt: float = 0.1,
                 logger: bool = False,
                 start_message_handler_from_init: bool = True,
                 dimension: int = 3) -> None:
        """
        Конструктор дочернего класса, наследующегося от Pio и Simulator.
        :param name: Имя дрона.
        :param mass: Масса дрона.
        :param position: Начальное состояние дрона вида [x, y, z, vx, vy, vz] или [x, y, vx, vy].
        Поле position имеет координаты и скорость, подобно сообщению LOCAL_POSITION_NED в mavlink.
        """
        DroneBase.__init__(self, ip, mavlink_port, name, mass, dimension, position, attitude, count_of_checking_points,
                           logger, dt=dt)  # Pio
        # Создание объекта Point3D
        self.simulation_objects = np.array([Point(mass, self._position[0:self.dimension],
                                                  self._position[self.dimension:self.dimension * 2])])
        self.connection_method = connection_method
        self.combine_system = combine_system
        self.count_of_checking_points = count_of_checking_points

        self.position_pid_matrix = np.array([
            [1.0] * self.dimension,
            [0.0] * self.dimension,
            [0.0] * self.dimension
        ], dtype=np.float64)

        self.velocity_pid_matrix = np.array([
            [3.0] * self.dimension,
            [0.0] * self.dimension,
            [0.1] * self.dimension
        ], dtype=np.float64)
        Simulator.__init__(self, self.simulation_objects, dt=dt, dimension=self.dimension)

        self.max_speed = 2
        # Период отправления следующего вектора скорости
        self.period_send_speed = 0.05
        self.speed_flag = True
        self._pid_position_controller = None
        self._pid_velocity_controller = None
        self.battery_voltage = 8
        self._heartbeat_send_time = time.time()
        self._heartbeat_timeout = 3
        # Границы симуляции
        self.lower_bound = np.array([-5.5, -5.5, 0])
        self.upper_bound = np.array([5.5, 5.5, 4])
        self.point_reached = False

        self._message_thread = None  # Поток для _message_handler
        self._handler_lock = threading.Lock()  # Мьютекс для синхронизации
        self.last_points = np.zeros((count_of_checking_points, self.dimension))
        if start_message_handler_from_init:
            self.start_message_handler()

    @property
    def position(self) -> Union[Array6, Array4]:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return np.hstack([self.simulation_objects[0].position, self.simulation_objects[0].speed])

    @position.setter
    def position(self, position: Union[Array6, Array4]) -> None:
        """
        Сеттер для _position
        :return: None
        """
        self.simulation_objects[0].position = position[0:self.dimension]
        self.simulation_objects[0].speed = position[self.dimension:self.dimension * 2]

    @property
    def speed(self) -> Union[Array2, Array3]:
        """
        Функция вернет скорость [vx, vy, vz]
        :return: Union[Array2, Array3]
        """
        return self.simulation_objects[0].speed

    def start_message_handler(self) -> None:
        """
        Запуск потока _message_handler.
        """
        if not self.simulation_turn_on:
            self.simulation_turn_on = True
            self._message_thread = threading.Thread(target=self._message_handler)
            self._message_thread.start()
            if self.logger:
                print("Message handler started.")

    def stop_message_handler(self) -> None:
        """
        Остановка потока _message_handler.
        """
        if self.simulation_turn_on:
            self.simulation_turn_on = False
            if self._message_thread:
                self._message_thread.join()
            if self.logger:
                print("Message handler stopped.")

    def _step_messege_handler(self):
        self.velocity_controller()
        for object_channel, simulation_object in enumerate(self.simulation_objects):
            self.step(simulation_object, object_channel)
        self.last_points = update_array(self.last_points, self.position[0:self.dimension])
        if self.logger:
            print(f"xyz = {self.position[0:self.dimension]}, "
                  f"speed = {self.position[self.dimension:self.dimension * 2]}, "
                  f"t_speed = {self.t_speed}")

    def _message_handler(self, *args):
        """
        Основной цикл обработки сообщений.
        """
        last_time = time.time()
        self._pid_position_controller = PIDController(*self.position_pid_matrix)
        self._pid_velocity_controller = PIDController(*self.velocity_pid_matrix)

        while self.simulation_turn_on:
            with self._handler_lock:  # Блокируем доступ для других операций
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    last_time = current_time
                    self._heartbeat_send_time = current_time
                    self._step_messege_handler()
                    self.position[0:self.dimension] = self.simulation_objects[0].position
                    self.position[self.dimension:self.dimension * 2] = self.simulation_objects[0].speed
                    if self.check_attitude_flag:
                        self.attitude_write()

            time.sleep(0.01)

    def velocity_controller(self):
        signal = self._pid_velocity_controller.compute_control(
            target_position=np.array(self.t_speed[0:self.dimension], dtype=np.float64),
            current_position=np.array(self.simulation_objects[0].speed, dtype=np.float64),
            dt=self.dt)
        self.set_force(signal, 0)

    def position_controller(self, position_xyz: Array3):
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(position_xyz, dtype=np.float64),
                current_position=self.simulation_objects[0].position,
                dt=self.dt),
            -self.max_speed,
            self.max_speed)
        self.t_speed = np.hstack([signal, 0])

    # Реализация обязательных методов абстрактного класса Pio
    def arm(self):
        if self.logger:
            print(f"{self.name} is armed.")

    def disarm(self):
        if self.logger:
            print(f"{self.name} is disarmed.")

    def takeoff(self):
        self.goto(self.position[0], self.position[1], 1.5, 0)
        if self.logger:
            print(f"{self.name} is taking off.")

    def land(self):
        self.goto(self.position[0], self.position[1], 0, 0)
        if self.logger:
            print(f"{self.name} is landing.")

    def goto(self,
             x: Union[float, int],
             y: Union[float, int],
             z: Union[float, int],
             yaw: Union[float, int],
             accuracy: Union[float, int] = 5e-2) -> None:
        """
        Функция берет целевую координату и вычисляет необходимые скорости для достижения целевой позиции, посылая их в
        управление t_speed.
        Для использования необходимо включить цикл v_while для посылки вектора скорости дрону.
        Максимальная скорость обрезается np.clip по полю self.max_speed.
        :param x: координата по x
        :type x: Union[float, int]
        :param y: координата по y
        :type: Union[float, int]
        :param z:  координата по z (не используется, если self.dimension = 2)
        :type: Union[float, int]
        :param yaw:  координата по yaw
        :type: Union[float, int]
        :param accuracy: Погрешность целевой точки 
        :type: Union[float, int]
        :return: None
        """
        if self.dimension == 2:
            target_point = [x, y]
        else:
            target_point = [x, y, z]

        with self._handler_lock:  # Захватываем управление
            last_time = time.time()
            self.point_reached = False
            self._pid_position_controller = PIDController(*self.position_pid_matrix)
            self._pid_velocity_controller = PIDController(*self.velocity_pid_matrix)
            while not self.point_reached:
                current_time = time.time()
                elapsed_time = current_time - last_time
                # Проверяем, прошло ли достаточно времени для очередного шага
                if elapsed_time >= self.dt:
                    self.point_reached = vector_reached(target_point,
                                                        self.last_points,
                                                        accuracy=accuracy)
                    self.position[0:self.dimension] = self.simulation_objects[0].position
                    self.position[self.dimension:self.dimension * 2] = self.simulation_objects[0].speed
                    self.last_points = update_array(self.last_points,
                                                    self.position[0:self.dimension])
                    self.velocity_controller()
                    self.position_controller(np.array(target_point))
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(self.simulation_objects):
                        self.step(simulation_object, object_channel)
                    if self.logger:
                        print(f"xyz = {self.position[0:self.dimension]}, "
                              f"speed = {self.position[self.dimension:self.dimension * 2]}, "
                              f"t_speed = {self.t_speed}")
                time.sleep(0.01)  # Даем CPU немного отдохнуть
            if self.logger:
                print(f"Точка {target_point} достигнута")
            self.t_speed = np.zeros(self.dimension + 1)

    def goto_from_outside(self,
                          x: Union[float, int],
                          y: Union[float, int],
                          z: Union[float, int],
                          yaw: Union[float, int],
                          accuracy: Union[float, int] = 5e-2) -> None:
        """
        Функция симулятор оригинальной функции в Pion, полностью повторяет функционал goto в данном классе
        :param x: координата по x
        :type x: Union[float, int]
        :param y: координата по y
        :type: Union[float, int]
        :param z:  координата по z
        :type: Union[float, int]
        :param yaw:  координата по yaw
        :type: Union[float, int]
        :param accuracy: Погрешность целевой точки
        :type: Union[float, int]
        :return: None
        """
        self.goto(x, y, z, yaw, accuracy)

    def stop(self):
        """
        Останавливает все потоки, завершает симуляцию
        """
        self.speed_flag = False
        self.simulation_turn_on = False
        print("Simulation stopped")

    def save_data(self,
                  file_name: str = 'data.npy',
                  path: str = '') -> None:
        """
        Функция для сохранения траектории в файл
        columns=['x', 'y', 'z', 'yaw', 'Vx', 'Vy', 'Vz', 'Vy_yaw', 'vxc', 'vyc', 'vzc', 'v_yaw_c', 't']
        :param file_name: название файла
        :type: str
        :param path: путь сохранения
        :type: str
        :return: None
        """
        self.stop()
        np.save(f'{path}{file_name}', self.trajectory[2:])

    def borders(self) -> None:

        """
        Функция накладывает границы симуляции для дрона
        :return: None

        """
        position = self.simulation_objects[0].position

        # Проверка на достижение границы и добавление отскока
        for i in range(self.dimension):
            if position[i] <= self.lower_bound[i]:
                position[i] += 0.1  # отскок внутрь области
                print("lower bound")
                self.point_reached = True  # Отменяем полетные цели
            elif position[i] >= self.upper_bound[i]:
                position[i] -= 0.1
                print("upper bound")
                self.point_reached = True

        # Применение ограничения с np.clip
        self.simulation_objects[0].position = np.clip(position, self.lower_bound, self.upper_bound)
