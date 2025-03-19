import threading
import time
from numpy.typing import NDArray
from typing import Union, Literal, Annotated, Any
import numpy as np
from .annotation import Array3, Array2


class Point:
    def __init__(self,
                 mass: float = 1.,
                 position: Union[Array2, Array3] = np.array([0, 0, 0], dtype=np.float64),
                 speed: Union[Array2, Array3] = np.array([0, 0, 0], dtype=np.float64),
                 trajectory_write: bool = False,
                 drag_coefficient: float = 0.01):  # Добавлен параметр для сопротивления воздуха
        """
        Инициализация объекта Point3D

        :param mass: масса объекта
        :type mass: Union[Array2, Array3]
        :param position: начальная позиция объекта в пространстве [x, y, z]
        :type position: Union[Array2, Array3]
        :param speed: начальная скорость объекта [vx, vy, vz]
        :type speed: Union[Array2, Array3]
        :param trajectory_write: записывать ли траекторию
        :type trajectory_write: bool
        :param drag_coefficient: коэффициент сопротивления воздуха
        :type drag_coefficient: float
        """
        if not position.shape == speed.shape:
            raise ValueError("Начальная координата должна иметь схожую размерность со своей скоростью")
        if position.shape not in [(2,), (3,)]:
            raise ValueError("Размерность точки должна быть равна 2 или 3")
        self.dimension: Annotated[int, Literal[2, 3]] = position.shape[0]
        self.trajectory: Trajectory_writer = Trajectory_writer(['x', 'y', 'vx', 'vy', 't'] if self.dimension == 2 else
                                            ['x', 'y', 'z', 'vx', 'vy', 'vz', 't'])
        self.trajectory_write: bool = trajectory_write
        self.mass: float = mass
        self.initial_position: Annotated[NDArray[Any], (self.dimension,)] = position
        self.position: Annotated[NDArray[Any], (self.dimension,)] = np.array(position, dtype=np.float64)
        self.speed: Annotated[NDArray[Any], (self.dimension,)] = np.array(speed, dtype=np.float64)
        self.time: float = 0.0
        self.drag_coefficient: float = drag_coefficient 

    def rk4_step(self,
                 acceleration: Union[Array2, Array3],
                 dt: float) -> None:
        """
        Шаг симуляции с использованием метода Рунге-Кутты 4-го порядка

        :param acceleration: вектор ускорения
        :type acceleration: Union[Array2, Array3]
        :param dt: временной шаг
        :type dt: float
        :return: None
        :rtype: None
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

    def step(self,
             force: Union[Array2, Array3],
             dt: float) -> None:
        """
        Шаг симуляции. Вычисляются следующие значения координат и скорости в зависимости от дискретного шага dt

        :param: force: сила воздействия на точку
        :type force: Union[Array2, Array3]
        :param dt: дискретный шаг времени
        :type dt: float
        :return: None
        :rtype: None
        """
        if force.shape != (self.dimension,):
            raise ValueError(f"Сила должна иметь размерность {self.dimension}, но имеет размерность:  {force.shape}")
        
        # Расчет силы сопротивления (линейное сопротивление)
        drag_force = -self.drag_coefficient * self.speed
        
        # Суммарная сила: внешняя сила + сила сопротивления
        total_force = force + drag_force
        
        # Используем сумму сил для расчета ускорения
        self.rk4_step(total_force / self.mass, dt=dt)
        self.time += dt
        if self.trajectory_write:
            self.trajectory.vstack(np.hstack([self.position, self.speed, self.time]))

    def get_trajectory(self) -> NDArray[np.float64]:
        """
        Функция возвращает траекторию точки

        :return: траектория точки
        :rtype: NDArray[np.float64]
        """
        return self.trajectory.get_trajectory()

class Simulator:
    """
    Класс симулятора, моделирующий поведение материальной точки
    """
    def __init__(self,
                 simulation_objects: NDArray[Any],
                 dt: float = 0.01,
                 dimension: int = 3):
        """
        Класс принимает в себя массив numpy с объектами, в которых реализован метод step, принимающий dt - дискретный
        шаг вычисления, а также в которых есть поля speed и position.
        Введем локальные определения: 
        канал объекта - порядковый номер объекта в self.simulation_object.
        """
        self.dimension: Annotated[int, Literal[2, 3]] = dimension
        self.simulation_objects: np.ndarray = simulation_objects
        self.simulation_turn_on: bool = False
        self.dt: float = dt
        self.forces: np.ndarray = np.zeros((np.shape(simulation_objects)[0], dimension))
        self.threading_list: list = []

    def start_simulation_while(self) -> None:
        """
        Функция запускает последовательную симуляцию всех симулируемых объектов

        :return: None
        :rtype: None
        """
        self.object_cycle('while')

    def start_simulation_for(self,
                             steps: int = 100) -> None:
        """
        Запускает симуляцию для всех объектов, используя цикл `for`

        :param steps: количество шагов симуляции
        :type steps: int
        :return: None
        :rtype: None
        """
        self.object_cycle('for', steps)

    def step(self,
             simulation_object: Point,
             object_channel: int) -> None:
        """
        Выполняет один шаг симуляции для объекта.

        :param simulation_object: объект, который выполняет шаг симуляции
        :type simulation_object: Point
        :param object_channel: канал, соответствующий объекту в массиве simulation_objects
        :type object_channel: int
        :return: None
        :rtype: None
        """
        simulation_object.step(self.forces[object_channel], self.dt)

    def set_force(self,
                  force: Array3,
                  object_channel: int) -> None:
        """
        Устанавливает силу для объекта на указанном канале.

        :param force: вектор силы, который будет применён к объекту
        :type force: Union[Array2, Array3]
        :param object_channel: канал, соответствующий объекту в массиве simulation_objects
        :type object_channel: int
        :return: None
        :rtype: None
        """
        self.forces[object_channel] = force

    def get_position(self) -> NDArray[np.float64]:
        """
        Возвращает матрицу позиций всех объектов симуляции.

        :return: матрица размером nx3, где n - количество объектов. Столбцы - x, y, z
        :rtype: np.ndarray
        """
        position_matrix = np.zeros((self.dimension,))
        for obj in self.simulation_objects:
            position_matrix = np.vstack([position_matrix, obj.position])
        return position_matrix[1:]

    def object_cycle(self,
                     type_of_cycle: str = 'while',
                     steps: int = 100) -> None:
        """
        Фукнция запускает симуляцию объекта.

        :param type_of_cycle: тип симуляции - while или for, если for, то нужно указать steps
        :type type_of_cycle: str
        :param steps: количество шагов симуляции для цикла for
        :type steps: int
        :return: None
        :rtype: None
        """
        if type_of_cycle == 'while':
            while self.simulation_turn_on:
                for object_channel, simulation_object in enumerate(self.simulation_objects):
                    self.step(simulation_object, object_channel)
        elif type_of_cycle == 'for':
            for _ in range(steps):
                for object_channel, simulation_object in enumerate(self.simulation_objects):
                    simulation_object.step(self.forces[object_channel], self.dt)
        else:
            print("Такой type_of_cycle не задан в начальных настройках, попробуйте while или for 1")


class Simulator_th(Simulator):
    """
    Класс запускает симуляции в отдельных потоках.
    """

    def start_simulation_while(self) -> None:
        """
        Функция запускает симуляцию всех объектов в self.simulation_object в отдельных потоках
        через цикл while с полем-флагом выключения self.simulation_turn_on

        :return: None
        :rtype: None
        """
        self.simulation_turn_on = True
        for channel, simulation_object in enumerate(self.simulation_objects):
            self.threading_list.append(
                threading.Thread(target=self.object_cycle, args=(simulation_object, channel, 'while')))
        for thread in self.threading_list:
            thread.start()

    def start_simulation_for(self,
                             steps: int = 100) -> None:
        """
        Запускает симуляцию для всех объектов в отдельных потоках, используя цикл `for`.

        :param steps: количество шагов симуляции
        :type steps: int
        :return: None
        :rtype: None
        """
        self.simulation_turn_on = True
        for channel, simulation_object in enumerate(self.simulation_objects):
            self.threading_list.append(
                threading.Thread(target=self.object_cycle, args=(simulation_object, channel, 'for', steps)))
        for thread in self.threading_list:
            thread.start()


class Simulator_realtime(Simulator):
    """
    Класс симуляции с синхронизацией с реальным временем
    """

    def object_cycle(self,
                     type_of_cycle: str = 'while',
                     steps: int = 100) -> None:
        """
        Фукнция запускает симуляцию объектов в отдельных потоках, но с
        синхронизацией с реальным временем.

        :param type_of_cycle: тип симуляции - while или for, если for, то нужно указать steps
        :type type_of_cycle: str
        :param steps: количество шагов симуляции для цикла for
        :return: None
        :rtype: None
        """
        last_time = time.time()
        if type_of_cycle == 'while':
            while self.simulation_turn_on:
                current_time = time.time()
                elapsed_time = current_time - last_time
                # Проверяем, прошло ли достаточно времени для очередного шага
                if elapsed_time >= self.dt:
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(self.simulation_objects):
                        self.step(simulation_object, object_channel)
        elif type_of_cycle == 'for':
            for _ in range(steps):
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(self.simulation_objects):
                        self.step(simulation_object, object_channel)
        else:
            print("Такой type_of_cycle не задан в начальных настройках, попробуйте while или for 1")


class Simulator_realtime_th(Simulator_realtime, Simulator_th):
    """
    Класс симуляции с синхронизацией в реальном времени и с запуском в отдельных потоках.
    """
    pass


class Trajectory_writer:
    def __init__(self,
                 list_of_names_columns: Union[list[str], NDArray[Any]]):
        """
        Специальный класс для записи и хранения траектории размером
        nx[len(list_of_names_columns)], n - количество точек.

        :param list_of_names_columns: названия колонн
        :type list_of_names_columns: Union[list[str], NDArray[Any]]
        """
        self.trajectory = np.zeros((len(list_of_names_columns),))
        self.columns = list_of_names_columns
        self.stopped = False

    def vstack(self,
               vstack_array: NDArray[np.float64]) -> None:
        """
        Функция объединяет входящие вектора с матрицей trajectory

        :param vstack_array: массив размерности len(list_of_names_columns)
        :type vstack_array: NDArray[np.float64]
        :return: None
        :rtype: None
        """
        if not self.stopped:
            self.trajectory = np.vstack([self.trajectory, vstack_array])

    def stop(self) -> None:
        """
        Остановка записи траектории
        """
        self.stopped = True

    def get_trajectory(self) -> NDArray[np.float64]:
        """
        Функция возвращает записанную траекторию

        :return: NDArray[np.float64]
        :rtype: None
        """
        if not self.trajectory.shape == (len(self.columns),):
            return self.trajectory[1:]
        else:
            return self.trajectory
