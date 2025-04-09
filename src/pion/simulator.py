import threading
import time
from typing import Annotated, Any, Literal, Union

import numpy as np
from numpy.typing import NDArray


class Point:
    def __init__(
        self,
        mass: float = 1.0,
        position: NDArray[np.float64] = np.array(
            [0, 0, 0, 0, 0, 0], dtype=np.float64
        ),
        trajectory_write: bool = False,
        drag_coefficient: float = 0.01,
    ):
        """
        Инициализация точки с трансляционным состоянием.

        Состояние задаётся вектором position = [x, y, z, vx, vy, vz].
        Все матричные операции выполняются без использования срезов.
        """
        if position.shape != (6,):
            raise ValueError(
                "position должен иметь размер 6 (x, y, z, vx, vy, vz)"
            )
        self.mass = mass
        self.drag_coefficient = drag_coefficient
        self.state: NDArray[np.float64] = np.array(position, dtype=np.float64)
        self.time: float = 0.0
        self.trajectory_write: bool = trajectory_write
        self.trajectory: Trajectory_writer = Trajectory_writer(
            ["x", "y", "z", "vx", "vy", "vz", "t"]
        )
        # Матрица динамики для state: d/dt[state] = A @ state + b,
        # где A = [0 I; 0 0]
        self.A = np.block(
            [
                [np.zeros((3, 3)), np.eye(3)],
                [np.zeros((3, 3)), np.zeros((3, 3))],
            ]
        )
        # Матрица для извлечения скорости: C @ state дает [vx, vy, vz]
        self.C = np.hstack([np.zeros((3, 3)), np.eye(3)])

    def step(self, force: NDArray[np.float64], dt: float) -> None:
        """
        Шаг симуляции для трансляционного движения.

        :param force: внешний вектор силы (3 элемента)
        :param dt: шаг по времени
        """
        # Расчёт силы сопротивления: -drag_coefficient * скорость,
        # скорость извлекается через матрицу C
        drag_force = -self.drag_coefficient * (self.C @ self.state)
        total_force = force + drag_force
        acceleration = total_force / self.mass  # (3,)
        # Формируем вектор b для динамики: b = [0,0,0, ax, ay, az]
        b = np.concatenate((np.zeros(3), acceleration))
        # Интегрирование методом Рунге-Кутты 4-го порядка (RK4)
        k1 = dt * (self.A @ self.state + b)
        k2 = dt * (self.A @ (self.state + 0.5 * k1) + b)
        k3 = dt * (self.A @ (self.state + 0.5 * k2) + b)
        k4 = dt * (self.A @ (self.state + k3) + b)
        self.state = self.state + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.time += dt
        if self.trajectory_write:
            self.trajectory.vstack(
                np.concatenate((self.state, np.array([self.time])))
            )

    def get_trajectory(self) -> NDArray[np.float64]:
        """Возвращает записанную траекторию."""
        return self.trajectory.get_trajectory()


class PointYaw(Point):
    def __init__(
        self,
        mass: float = 1.0,
        # Трансляционное состояние задаётся как [x, y, z, vx, vy, vz]
        position: NDArray[np.float64] = np.array(
            [0, 0, 0, 0, 0, 0], dtype=np.float64
        ),
        attitude: NDArray[np.float64] = np.array(
            [0, 0, 0, 0, 0, 0], dtype=np.float64
        ),
        trajectory_write: bool = False,
        drag_coefficient: float = 0.01,
        yaw: float = 0.0,
    ):
        """
        Инициализация точки с трансляционным и угловым состоянием.

        Трансляционное состояние задаётся параметром position = [x, y, z, vx, vy, vz].
        Угловое состояние (attitude) хранится в векторе:
            [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate].
        При симуляции обновляется только динамика yaw.
        """
        super().__init__(mass, position, trajectory_write, drag_coefficient)
        self.attitude: NDArray[np.float64] = attitude
        self.attitude[2] = yaw
        # Матрица динамики для attitude: обновляется только yaw посредством зависимости от yaw_rate.
        self.A_att = np.zeros((6, 6))
        self.A_att[2, 5] = 1.0
        self.E_trans = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        self.E_yaw = np.array([[0, 0, 0, 1]])

    def step(self, force: NDArray[np.float64], dt: float) -> None:
        """
        Шаг симуляции для точки с угловым состоянием.

        :param force: управляющий вектор из 4 элементов:
                      первые 3 – для трансляционного движения,
                      4-й – для управления yaw (угловое ускорение)
        :param dt: шаг по времени
        """
        translational_force = self.E_trans @ force
        super().step(translational_force, dt)

        yaw_input = (self.E_yaw @ force).item()
        b_att = np.concatenate((np.zeros(5), np.array([yaw_input])))
        k1a = dt * (self.A_att @ self.attitude + b_att)
        k2a = dt * (self.A_att @ (self.attitude + 0.5 * k1a) + b_att)
        k3a = dt * (self.A_att @ (self.attitude + 0.5 * k2a) + b_att)
        k4a = dt * (self.A_att @ (self.attitude + k3a) + b_att)
        self.attitude = self.attitude + (k1a + 2 * k2a + 2 * k3a + k4a) / 6

        self.time += dt
        if self.trajectory_write:
            E_att_yaw = np.array([[0, 0, 1, 0, 0, 0]])
            yaw_value = (E_att_yaw @ self.attitude).item()
            self.trajectory.vstack(
                np.concatenate((self.state, np.array([yaw_value, self.time])))
            )

    def get_attitude(self) -> NDArray[np.float64]:
        """Возвращает текущее состояние attitude."""
        return self.attitude


class Simulator:
    """
    Класс симулятора, моделирующий поведение материальной точки
    """

    def __init__(
        self,
        simulation_objects: NDArray[Any],
        dt: float = 0.01,
        dimension: int = 3,
    ):
        """
        Класс принимает в себя массив numpy с объектами,

        в которых реализован метод step, принимающий dt - дискретный
        шаг вычисления, а также в которых есть поля speed и position.
        Введем локальные определения:
        канал объекта - порядковый номер объекта в self.simulation_object.
        """
        self.dimension: Annotated[int, Literal[2, 3]] = dimension
        self.simulation_objects: np.ndarray = simulation_objects
        self.simulation_turn_on: bool = False
        self.dt: float = dt
        force_dim = (
            dimension + 1
            if hasattr(simulation_objects[0], "E_yaw")
            else dimension
        )
        self.forces: np.ndarray = np.zeros(
            (np.shape(simulation_objects)[0], force_dim)
        )
        self.threading_list: list = []

    def start_simulation_while(self) -> None:
        """
        Метод запускает последовательную симуляцию всех симулируемых объектов

        :return: None
        :rtype: None
        """
        self.object_cycle("while")

    def start_simulation_for(self, steps: int = 100) -> None:
        """
        Запускает симуляцию для всех объектов, используя цикл `for`

        :param steps: количество шагов симуляции
        :type steps: int
        :return: None
        :rtype: None
        """
        self.object_cycle("for", steps)

    def step(self, simulation_object: Point, object_channel: int) -> None:
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

    def set_force(self, force: NDArray, object_channel: int) -> None:
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

    def object_cycle(
        self, type_of_cycle: str = "while", steps: int = 100
    ) -> None:
        """
        Фукнция запускает симуляцию объекта.

        :param type_of_cycle: тип симуляции - while или for, если for, то нужно указать steps
        :type type_of_cycle: str
        :param steps: количество шагов симуляции для цикла for
        :type steps: int
        :return: None
        :rtype: None
        """
        if type_of_cycle == "while":
            while self.simulation_turn_on:
                for object_channel, simulation_object in enumerate(
                    self.simulation_objects
                ):
                    self.step(simulation_object, object_channel)
        elif type_of_cycle == "for":
            for _ in range(steps):
                for object_channel, simulation_object in enumerate(
                    self.simulation_objects
                ):
                    simulation_object.step(
                        self.forces[object_channel], self.dt
                    )
        else:
            print(
                "Такой type_of_cycle не задан в начальных настройках, попробуйте while или for 1"
            )


class Simulator_th(Simulator):
    """
    Класс запускает симуляции в отдельных потоках.
    """

    def start_simulation_while(self) -> None:
        """
        Метод запускает симуляцию всех объектов в self.simulation_object

        в отдельных потоках через цикл while с полем-флагом
        выключения self.simulation_turn_on

        :return: None
        :rtype: None
        """
        self.simulation_turn_on = True
        for channel, simulation_object in enumerate(self.simulation_objects):
            self.threading_list.append(
                threading.Thread(
                    target=self.object_cycle,
                    args=(simulation_object, channel, "while"),
                )
            )
        for thread in self.threading_list:
            thread.start()

    def start_simulation_for(self, steps: int = 100) -> None:
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
                threading.Thread(
                    target=self.object_cycle,
                    args=(simulation_object, channel, "for", steps),
                )
            )
        for thread in self.threading_list:
            thread.start()


class Simulator_realtime(Simulator):
    """
    Класс симуляции с синхронизацией с реальным временем
    """

    def object_cycle(
        self, type_of_cycle: str = "while", steps: int = 100
    ) -> None:
        """
        Фукнция запускает симуляцию объектов в отдельных потоках, но с синхронизацией с реальным временем.

        :param type_of_cycle: тип симуляции - while или for, если for, то нужно указать steps
        :type type_of_cycle: str
        :param steps: количество шагов симуляции для цикла for
        :return: None
        :rtype: None
        """
        last_time = time.time()
        if type_of_cycle == "while":
            while self.simulation_turn_on:
                current_time = time.time()
                elapsed_time = current_time - last_time
                # Проверяем, прошло ли достаточно времени для очередного шага
                if elapsed_time >= self.dt:
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(
                        self.simulation_objects
                    ):
                        self.step(simulation_object, object_channel)
        elif type_of_cycle == "for":
            for _ in range(steps):
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(
                        self.simulation_objects
                    ):
                        self.step(simulation_object, object_channel)
        else:
            print(
                "Такой type_of_cycle не задан в начальных настройках, попробуйте while или for 1"
            )


class Simulator_realtime_th(Simulator_realtime, Simulator_th):
    """
    Класс симуляции с синхронизацией в реальном времени и с запуском в отдельных потоках.
    """

    pass


class Trajectory_writer:
    def __init__(self, list_of_names_columns: Union[list[str], NDArray[Any]]):
        """
        Специальный класс для записи и хранения траектории размером nx[len(list_of_names_columns)], n - количество точек.

        :param list_of_names_columns: названия колонн
        :type list_of_names_columns: Union[list[str], NDArray[Any]]
        """
        self.trajectory = np.zeros((len(list_of_names_columns),))
        self.columns = list_of_names_columns
        self.stopped = False

    def vstack(self, vstack_array: NDArray[np.float64]) -> None:
        """
        Метод объединяет входящие вектора с матрицей trajectory

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
        Метод возвращает записанную траекторию

        :return: NDArray[np.float64]
        :rtype: None
        """
        if not self.trajectory.shape == (len(self.columns),):
            return self.trajectory[1:]
        else:
            return self.trajectory
