import numpy as np
from typing import List, Union, Optional
import time


class Point3D:
    def __init__(self,
                 mass: float = 0.3,
                 position: Union[List[float], np.ndarray] = None,
                 speed: Union[List[float], np.ndarray] = None) -> None:
        """
        Инициализация объекта Point3D.

        :param mass: Масса объекта.
        :param position: Начальная позиция объекта в пространстве [x, y, z].
        :param speed: Начальная скорость объекта [vx, vy, vz].
        """
        if speed is None:
            speed = [0, 0, 0]
        if position is None:
            position = [0, 0, 0]
        self.mass = mass
        self.initial_position = np.array(position, dtype=np.float64)
        self.position = np.array(position, dtype=np.float64)
        self.speed = np.array(speed, dtype=np.float64)

    def rk4_step(self,
                 acceleration: np.ndarray,
                 dt: float) -> None:
        """
        Шаг симуляции с использованием метода Рунге-Кутты 4-го порядка.

        :param acceleration: Вектор ускорения.
        :param dt: Временной шаг.
        """
        k1v = acceleration * dt
        k1x = self.speed * dt

        k2v = (acceleration + 0.5 * k1v) * dt
        k2x = (self.speed + 0.5 * k1x) * dt

        k3v = (acceleration + 0.5 * k2v) * dt
        k3x = (self.speed + 0.5 * k2x) * dt

        k4v = (acceleration + k3v) * dt
        k4x = (self.speed + k3x) * dt

        self.speed += (k1v + 2 * k2v + 2 * k3v + k4v) / 6
        self.position += (k1x + 2 * k2x + 2 * k3x + k4x) / 6


class Simulator3D:
    def __init__(self,
                 simulation_object: Optional['Point3D'] = None,
                 name: str = 'simulator',
                 mass: float = 0.3,
                 position: Union[List[float], np.ndarray] = None,
                 speed: Union[List[float], np.ndarray] = None,
                 dt: float = 0.1) -> None:
        if speed is None:
            speed = [0, 0, 0]
        if position is None:
            position = [0, 0, 0]
        self.name = name
        self.dt = dt  # Временной шаг для симуляции
        if simulation_object is not None:
            self.simulation_object = simulation_object
        else:
            self.simulation_object = Point3D(mass, position, speed),
        self.simulation_time = 0  # Инициализируем симулированное время
        self.external_control_signal = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.time_data, self.x_data, self.y_data, self.z_data = [], [], [], []

    def receive_external_signal(self,
                                control_signal):
        """
        Метод для получения внешнего управляющего сигнала.
        Например, этот метод можно использовать для получения команды из внешнего источника (сети, сенсора, и т.д.).
        control_signal: np.array([x, y, z]) — вектор управления.
        """
        self.external_control_signal = control_signal

    def step(self) -> None:
        """
        Выполняет один шаг симуляции, обновляя позицию и скорость объекта.
        """
        # Добавляем внешнее управление
        total_control_signal = self.external_control_signal

        # Вычисляем ускорение
        acceleration = total_control_signal / self.simulation_object.mass
        self.simulation_object.rk4_step(acceleration, self.dt)

        # Обновляем симулированное время и сохраняем данные для анализа
        self.simulation_time += self.dt
        self.time_data.append(self.simulation_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def run_simulation(self,
                       steps: int) -> None:
        """
        Запуск симуляции на определенное количество шагов.

        :param steps: Количество шагов симуляции.
        """
        for _ in range(steps):
            self.step()


class Simulator3DRealTime(Simulator3D):
    def __init__(self,
                 simulation_object: Optional['Point3D'] = None,
                 name: str = 'simulator real time',
                 mass: float = 0.3,
                 position: Union[List[float], np.ndarray] = None,
                 speed: Union[List[float], np.ndarray] = None,
                 dt: float = 0.1) -> None:
        """
        Инициализация симулятора для работы в реальном времени.

        :param name: Имя симуляции.
        :param mass: Масса объекта.
        :param position: Начальная позиция объекта.
        :param speed: Начальная скорость объекта.
        :param dt: Временной шаг.
        :param max_acceleration: Максимальное ускорение.
        """
        # Инициализируем базовый класс
        if speed is None:
            speed = [0, 0, 0]
        if position is None:
            position = [0, 0, 0]
        super().__init__(simulation_object, name, mass, position, speed, dt)

    def run_real_time_simulation(self) -> None:
        """
        Запуск симуляции в реальном времени. Шаг симуляции синхронизируется с реальным временем.
        """
        last_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - last_time

            # Проверяем, прошло ли достаточно времени для очередного шага
            if elapsed_time >= self.dt:
                self.step()
                last_time = current_time

            time.sleep(0.01)  # Немного ждем, чтобы избежать слишком частых проверок
            print(f"Position: {self.simulation_object.position}, Time: {self.simulation_time}")
