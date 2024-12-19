from .simulator import Simulator, Point
from .pio import Pio
from .functions import vector_reached
from pion.cython_pid import PIDController  # Cython-версия PIDController
from typing import List, Union
from .annotation import *
import numpy as np
import time
import threading


class Spion(Simulator, Pio):
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 connection_method: str = 'udpout',
                 combine_system: int = 0,
                 count_of_checking_points: int = 20,
                 name: str = 'simulator',
                 mass: float = 0.3,
                 position: Union[Array6, None] = None,
                 attitude: Union[Array6, None] = None,
                 dt: float = 0.1,
                 logger: bool = False,
                 start_message_handler_from_init: bool = True) -> None:
        """
        Конструктор дочернего класса, наследующегося от Pio и Simulator.
        :param name: Имя дрона.
        :param mass: Масса дрона.
        :param position: Начальное состояние дрона вида [x, y, z, vx, vy, vz]
        """
        self.logger = logger
        if position is None:
            position = np.array([0, 0, 0, 0, 0, 0])
        if attitude is None:
            attitude = np.array([0, 0, 0, 0, 0, 0])
        self.ip = ip
        self.mavlink_port = mavlink_port
        self.connection_method = connection_method
        self.combine_system = combine_system
        self.count_of_checking_points = count_of_checking_points
        self.t0 = time.time()
        # Создание объекта Point3D
        self.simulation_objects = np.array([Point(mass, position[0:3], position[3:6])])
        Simulator.__init__(self, self.simulation_objects, dt=dt, dimension=3)
        Pio.__init__(self)  # Pio
        # Инициализация дополнительных параметров, специфичных для дрона
        self.name = name
        self.mass = mass
        self._position = position
        self._attitude = attitude
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw]
        self.t_speed = np.array([0, 0, 0, 0])
        self.max_speed = 2
        # Период отправления следующего вектора скорости
        self.period_send_speed = 0.05
        self.speed_flag = True
        self.pid_position_controller = PIDController(np.array([1., 1., 1.], dtype=np.float64),
                                                     np.array([0., 0., 0.], dtype=np.float64),
                                                     np.array([0., 0., 0.], dtype=np.float64))
        self.pid_velocity_controller = PIDController(np.array([3., 3., 3.], dtype=np.float64),
                                                     np.array([0., 0., 0.], dtype=np.float64),
                                                     np.array([0.1, 0.1, 0.1], dtype=np.float64))
        self.battery_voltage = 8
        self._heartbeat_send_time = time.time()
        # Информация, включающая
        # x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t
        # которая складывается в матрицу (n, 17), где n - число измерений
        self.trajectory = np.zeros((2, 17))
        # Границы симуляции
        self.lower_bound = np.array([-5.5, -5.5, 0])
        self.upper_bound = np.array([5.5, 5.5, 4])
        self.point_reached = False
        self.check_attitude_flag = True
        self._message_thread = None  # Поток для _message_handler
        self._handler_lock = threading.Lock()  # Мьютекс для синхронизации
        if start_message_handler_from_init:
            self.start_message_handler()

    @property
    def position(self) -> Array6:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return np.hstack([self.simulation_objects[0].position, self.simulation_objects[0].speed])

    @position.setter
    def position(self, position: Array6) -> None:
        """
        Сеттер для _position
        :return: None
        """
        self.simulation_objects[0].position = position[0:3]
        self.simulation_objects[0].speed = position[3:6]

    @property
    def attitude(self) -> Array6:
        """
        Функция вернет ndarray (6,) с координатами roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
        :return: np.ndarray
        """
        return self._attitude

    @attitude.setter
    def attitude(self, attitude: Array6) -> None:
        """
        Сеттер для _attitude
        :return: None
        """
        self._attitude = attitude

    def start_message_handler(self) -> None:
        """
        Запуск потока _message_handler.
        """
        if not self.simulation_turn_on:
            self.simulation_turn_on = True
            self._message_thread = threading.Thread(target=self._message_handler, daemon=True)
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
        if self.logger:
            print(f"xyz = {self.position[0:3]}, speed = {self.position[3:6]}, t_speed = {self.t_speed}")



    def _message_handler(self, *args):
        """
        Основной цикл обработки сообщений.
        """
        last_time = time.time()
        while self.simulation_turn_on:
            with self._handler_lock:  # Блокируем доступ для других операций
                self.position[0:3] = self.simulation_objects[0].position
                self.position[3:6] = self.simulation_objects[0].speed
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    last_time = current_time
                    self._heartbeat_send_time = current_time
                    if self.check_attitude_flag:
                        self.attitude_write()

                self._step_messege_handler()
            time.sleep(0.01)

    def velocity_controller(self):
        # print(f"Spion \n target: {self.t_speed[0:3]}, current = {np.array(self.simulation_objects[0].speed, dtype=np.float64)}, dt = {self.dt}")
        signal = self.pid_velocity_controller.compute_control(
            target_position=np.array(self.t_speed[0:3], dtype=np.float64),
            current_position=np.array(self.simulation_objects[0].speed, dtype=np.float64),
            dt=self.dt)
        self.set_force(signal, 0)

    def position_controller(self, position_xyz: Array3):
        signal = np.clip(
            self.pid_position_controller.compute_control(
                target_position=np.array(position_xyz, dtype=np.float64),
                current_position=self.simulation_objects[0].position,
                dt=self.dt),
            -self.max_speed,
            self.max_speed)
        self.t_speed = np.hstack([signal, 0])

    # Реализация обязательных методов абстрактного класса Pio
    def arm(self):
        print(f"{self.name} is armed.")

    def disarm(self):
        print(f"{self.name} is disarmed.")

    def takeoff(self):
        self.goto(self.position[0], self.position[1], 1.5, 0)
        print(f"{self.name} is taking off.")

    def land(self):
        self.goto(self.position[0], self.position[1], 0, 0)
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
        :param z:  координата по z
        :type: Union[float, int]
        :param yaw:  координата по yaw
        :type: Union[float, int]
        :param accuracy: Погрешность целевой точки 
        :type: Union[float, int]
        :return: None
        """
        with self._handler_lock:  # Захватываем управление
            last_time = time.time()
            self.point_reached = False
            while not self.point_reached:
                current_time = time.time()
                elapsed_time = current_time - last_time
                # Проверяем, прошло ли достаточно времени для очередного шага
                if elapsed_time >= self.dt:
                    self.point_reached = vector_reached([x, y, z],
                                                        self.simulation_objects[0].position[0:3],
                                                        accuracy=accuracy)
                    self.position[0:3] = self.simulation_objects[0].position
                    self.position[3:6] = self.simulation_objects[0].speed

                    self.velocity_controller()
                    self.position_controller(np.array([x, y, z]))
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(self.simulation_objects):
                        self.step(simulation_object, object_channel)
                    if self.logger:
                        print(f"xyz = {self.position[0:3]}, speed = {self.position[3:6]}, t_speed = {self.t_speed}")
                time.sleep(0.01)  # Даем CPU немного отдохнуть
            if self.logger:
                print(f"Точка {x, y, z} достигнута")
            self.t_speed = np.array([0, 0, 0, 0])

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

    def set_v(self,
              ampl: Union[float, int] = 1) -> None:
        """
        (Имитация)
        Создает поток, который вызывает функцию v_while() для параллельной отправки вектора скорости
        :param ampl: Амплитуда усиления вектора скорости
        :type ampl: float | int
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
        if not np.allclose(stack[:-1], self.trajectory[-2, :-1]):
            self.trajectory = np.vstack([self.trajectory, stack])

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
        self.speed_flag = False
        self.simulation_turn_on = False
        np.save(f'{path}{file_name}', self.trajectory[2:])

    def borders(self) -> None:

        """
        Функция накладывает границы симуляции для дрона
        :return: None

        """
        position = self.simulation_objects[0].position

        # Проверка на достижение границы и добавление отскока
        for i in range(3):
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

    def led_control(self,
                    led_id=255,
                    r=0,
                    g=0,
                    b=0) -> None:
        """
        Функция имитация.
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
        return None

    def goto_yaw(self,
                 yaw: Union[float, int] = 0,
                 accuracy: Union[float, int] = 0.087) -> None:
        """
        Функция пробка.
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
