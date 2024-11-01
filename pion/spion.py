from .simulator import Simulator3DRealTime, Point3D
from .pio import Pio
from .functions import vector_reached
# from .controller import PIDController
from pion.cython_pid import PIDController  # Cython-версия PIDController
from typing import List, Union, Optional, overload
import numpy as np
import time
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import combinations, product
import matplotlib
matplotlib.use('Agg')


class Spion(Simulator3DRealTime, Pio):
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 connection_method: str = 'udpout',
                 combine_system: int = 0,
                 count_of_checking_points: int = 20,
                 name: str = 'simulator',
                 mass: float = 0.3,
                 position: Union[List[float], np.ndarray] = None,
                 speed: Union[List[float], np.ndarray] = None,
                 dt: float = 0.1) -> None:
        """
        Конструктор дочернего класса, наследующегося от Pio и Simulator3DRealTime.
        :param name: Имя дрона.
        :param mass: Масса дрона.
        :param position: Начальная позиция дрона.
        :param simulation_speed: Скорость симуляции.
        """
        if speed is None:
            speed = [0, 0, 0]
        if position is None:
            position = [0, 0, 0]
        self.ip = ip
        self.mavlink_port = mavlink_port
        self.connection_method = connection_method
        self.combine_system = combine_system
        self.count_of_checking_points = count_of_checking_points
        self.t0 = time.time()
        # Создание объекта Point3D
        self.simulation_object = Point3D(mass, position, speed)
        Simulator3DRealTime.__init__(self, self.simulation_object, name, mass, position, speed, dt)
        Pio.__init__(self)  # Pio
        # Инициализация дополнительных параметров, специфичных для дрона
        self.name = name
        self.mass = mass
        self.position = position
        self._attitude = np.array([0, 0, 0, 0, 0, 0])
        self._position = np.array([0, 0, 0, 0, 0, 0])
        # Список потоков
        self.threads = []
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
        self.t_speed = np.array([0, 0, 0, 0])
        self.max_speed = 2
        # Период отправления следующего вектора скорости
        self.period_send_speed = 0.05
        self.speed_flag = True
        self.pid_controller = PIDController(np.array([10, 10, 10], dtype=np.float64), 
                                            np.array([0, 0, 0], dtype=np.float64), 
                                            np.array([1, 1, 1], dtype=np.float64))
        self.battery_voltage = 8
        self._heartbeat_send_time = time.time()
        # Информация, включающая
        # x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t
        # которая складывается в матрицу (n, 17), где n - число измерений
        self.trajectory = np.zeros((2, 17))



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
        Функция берет целевую координату и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.
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
        # pid_controller = PIDController(np.array([1, 1, 1], dtype=np.float64), np.array([0, 0, 0], dtype=np.float64), np.array([1, 1, 1], dtype=np.float64))
        point_reached = False
        dt = time.time()
 
        while not point_reached:
            dt = time.time() - dt
            point_reached = vector_reached([x, y, z], self.simulation_object.position[0:3], accuracy=accuracy)
            signal = self.pid_controller.compute_control(
            target_position=np.array([x, y, z], dtype=np.float64),
            current_position=self.simulation_object.position,
            dt=self.dt)
            self.t_speed = np.hstack([np.clip(signal,
            -self.max_speed, self.max_speed), 0])
            time.sleep(self.period_send_speed)
            print(f"local to {x, y, z, yaw}, current_pos = {self.position[0:3]}, mass = {self.mass}, signal = {signal}\n")
        self.t_speed = np.array([0, 0, 0, 0])
        self.external_control_signal = self.t_speed[0:3]
        self.simulation_object.speed = np.array(self.t_speed[0:3], dtype=np.float64)

        print(f"{self.name} is moving to {self.position}.")

    def set_v(self,
              ampl: Union[float, int] = 1) -> None:
        """
        Создает поток, который вызывает функцию v_while() для параллельной отправки вектора скорости
        :param ampl: Амплитуда усиления вектора скорости
        :type ampl: float | int
        :return: None
        """
        self.speed_flag = True
        self.threads.append(threading.Thread(target=self.run_real_time_simulation))
        self.threads[-1].start()

    def attitude_write(self) -> None:
        """
        Функция для записи траектории в numpy массив. Записывается только уникальная координата
        :return:
        """
        t = time.time() - self.t0
        stack = np.hstack([self.position, self.attitude, self.t_speed, [t]])
        if not np.all(np.equal(stack[:-1], self.trajectory[-2, :-1])):
            self.trajectory = np.vstack([self.trajectory, stack])

    def run_real_time_simulation(self) -> None:
        """
        Запуск симуляции в реальном времени. Шаг симуляции синхронизируется с реальным временем.
        """
        last_time = time.time()
        while self.speed_flag:
            self.borders()
            current_time = time.time()
            elapsed_time = current_time - last_time
            self._heartbeat_send_time = current_time

            # Проверяем, прошло ли достаточно времени для очередного шага
            if elapsed_time >= self.dt:
                self.step()
                last_time = current_time
            self._position = np.hstack([self.simulation_object.position, self.simulation_object.speed])
            self.external_control_signal = self.t_speed[0:3]
                    # x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t
            self.attitude_write()
            time.sleep(0.01)  # Немного ждем, чтобы избежать слишком частых проверок
            # print(f"Position: {self.simulation_object.position}, Time: {self.simulation_time}, speed_flag = {self.speed_flag}")

    def stop(self):
        """
        Останавливает все потоки, завершает симуляцию
        """
        self.speed_flag = False
        # for thread in self.threads:
        #     thread.join()  # Ждем завершения всех потоков
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
        np.save(f'{path}{file_name}', self.trajectory[2:])
    
    def borders(self):
        lower_bound = np.array([-5.5, -5.5, 0])
        upper_bound = np.array([5.5, 5.5, 4])
        position = self.simulation_object.position

        # Проверка на достижение границы и добавление отскока
        for i in range(3):
            if position[i] <= lower_bound[i]:
                position[i] += 0.1  # отскок внутрь области
            elif position[i] >= upper_bound[i]:
                position[i] -= 0.1

        # Применение ограничения с np.clip
        self.simulation_object.position = np.clip(position, lower_bound, upper_bound)



    def led_control(self,
                led_id=255,
                r=0,
                g=0,
                b=0) -> None:
        """
        Функция пробка.
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




