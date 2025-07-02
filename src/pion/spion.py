import threading
import time
from typing import Annotated, Any, Optional, Union

import numpy as np
from numpy.typing import NDArray

from pion.cython_pid import PIDController  # Cython-версия PIDController
from pionfunc.annotation import Array3, Array4, Array6
from pionfunc.functions import start_threading, update_array, vector_reached

from .pio import DroneBase
from .simulator import PointYaw, Simulator

def normalize_channel(value: int) -> float:
    """
    Функция нормализации rc-каналов
    """
    return np.clip((value - 1500) / 500, -1.0, 1.0)

class Spion(Simulator, DroneBase):
    """
    Класс симулятор, повторяющий действия Pion в симуляции математической модели точки
    """

    def __init__(
        self,
        ip: str = "10.1.100.114",
        mavlink_port: int = 5656,
        connection_method: str = "udpout",
        position: NDArray[np.float64] = np.array(
            [0, 0, 0, 0, 0, 0], dtype=np.float64
        ),
        attitude: NDArray[np.float64] = np.array(
            [0, 0, 0, 0, 0, 0], dtype=np.float64
        ),
        combine_system: int = 0,
        count_of_checking_points: int = 2,
        name: str = "simulator",
        mass: float = 0.3,
        dt: float = 0.1,
        logger: bool = False,
        checking_components: bool = True,
        accuracy: float = 2e-5,
        max_speed: float = 2.0,
        start_message_handler_from_init: bool = True,
    ) -> None:
        """
        Конструктор дочернего класса, наследующегося от Pio и Simulator

        :param ip: IP-адрес для подключения к дрону
        :type ip: str

        :param mavlink_port: Порт для MAVLink соединения
        :type mavlink_port: int

        :param connection_method: Метод соединения, например, 'udpout' для MAVLink.
        :type connection_method: str

        :param position: Начальное состояние дрона вида [x, y, z, vx, vy, vz] или [x, y, vx, vy]
        :type position: Union[Array6, Array4, None]

        :param attitude: Начальное состояние дрона вида [roll, pitch, yaw, v_roll, v_pitch, v_yaw]
        :type attitude: Union[Array6, None]

        :param combine_system: Системный код для комбинированной системы управления: 1, 2, 3
        :type combine_system: int

        :param count_of_checking_points: Количество последних точек, используемых для проверки достижения цели.
        :type count_of_checking_points: int

        :param name: Название экземпляра
        :type name: str

        :param mass: Масса дрона
        :type mass: float

        :param dt: Период приема всех сообщений с дрона
        :type dt: float

        :param logger: Включить логирование
        :type logger: bool

        :param checking_components: Параметр для проверки номеров компонентов. Отключается для в сторонних симуляторах
         во избежание ошибок
        :type checking_components: bool

        :param accuracy: Максимальное отклонение от целевой позиции для функции goto_from_outside
        :type accuracy: float

        :param max_speed: Максимальная скорость дрона в режиме управления по скорости
        :type max_speed: float

        :param start_message_handler_from_init: Старт message handler при создании объекта
        :type start_message_handler_from_init: bool
        """
        DroneBase.__init__(
            self,
            ip=ip,
            mavlink_port=mavlink_port,
            name=name,
            mass=mass,
            position=position,
            attitude=attitude,
            count_of_checking_points=count_of_checking_points,
            logger=logger,
            checking_components=checking_components,
            accuracy=accuracy,
            dt=dt,
            max_speed=max_speed,
        )
        self.__goto_process = False
        initial_yaw = self._attitude[2]
        self.simulation_objects: NDArray[Any] = np.array(
            [
                PointYaw(
                    mass=mass,
                    position=self._position,
                    attitude=attitude,
                    trajectory_write=False,
                    drag_coefficient=0.01,
                    yaw=initial_yaw,
                )
            ]
        )
        self.connection_method: str = connection_method
        self.combine_system: int = combine_system
        self.count_of_checking_points: int = count_of_checking_points

        self.position_pid_matrix: Annotated[NDArray[Any], (3, 4)] = np.array(
            [
                [2.0] * 4,
                [0.0] * 4,
                [0.1] * 4,
            ],
            dtype=np.float64,
        )

        self.velocity_pid_matrix: Annotated[NDArray[Any], (3, 4)] = np.array(
            [
                [3.0] * 4,
                [0.0] * 4,
                [0.1] * 4,
            ],
            dtype=np.float64,
        )
        Simulator.__init__(self, self.simulation_objects, dt=dt, dimension=3)
        # Период отправления следующего вектора скорости
        self.period_send_speed: float = 0.05
        self.speed_flag: bool = True
        self._pid_velocity_controller: Optional[PIDController] = None
        self._heartbeat_send_time: float = time.time()
        self._heartbeat_timeout: float = 3.0
        # Границы симуляции
        self.lower_bound: Array3 = np.array([-5.5, -5.5, 0])
        self.upper_bound: Array3 = np.array([5.5, 5.5, 4])
        self.point_reached: bool = False
        self._message_thread: Optional[threading.Thread] = None
        self.last_points: Annotated[
            NDArray[Any], (count_of_checking_points, 4)
        ] = np.zeros((count_of_checking_points, 4))
        if start_message_handler_from_init:
            self.start_message_handler()

    @property
    def position(self) -> NDArray[np.float64]:
        """
        Возвращает объединённый вектор: [x, y, z, vx, vy, vz]
        """
        return self.simulation_objects[0].state

    @position.setter
    def position(self, pos: Array6) -> None:
        """
        Сеттер для обновления трансляционного состояния.
        """
        if pos.shape[0] < 6:
            raise ValueError("position должен содержать минимум 6 элементов")
        self.simulation_objects[0].state = pos[0:6]

    @property
    def speed(self) -> NDArray[np.float64]:
        """
        Возвращает скорость как [vx, vy, vz] из state.
        """
        return self.simulation_objects[0].state[3:6]

    @property
    def attitude(self) -> Array6:
        """
        Метод вернет ndarray (6,) с координатами roll, pitch, yaw, rollspeed, pitchspeed, yawspeed

        :return: np.ndarray
        """
        return self.simulation_objects[0].attitude

    def takeoff(self) -> None:
        """
        Метод взлета дрона

        :return: None
        :rtype: None
        """
        super().takeoff()
        self.goto(
            self.position[0],
            self.position[1],
            1.5,
            self.yaw,
        )

    def land(self) -> None:
        """
        Метод посадки дрона

        :return: None
        :rtype: None
        """
        super().land()
        self.goto(self.position[0], self.position[1], 0, self.yaw)

    def start_message_handler(self) -> None:
        """
        Запуск потока _message_handler

        :return: None
        :rtype: None
        """
        if not self.simulation_turn_on:
            self.simulation_turn_on = True
            self._message_thread = threading.Thread(
                target=self._message_handler
            )
            self.threads.append(self._message_thread)
            self._message_thread.start()
            if self.logger:
                self.logs.update({"Status": "Message handler started"})

    def stop_message_handler(self) -> None:
        """
        Остановка потока _message_handler

        :return: None
        :rtype: None
        """
        if self.simulation_turn_on:
            self.simulation_turn_on = False
            if self._message_thread:
                self._message_thread.join()
            if self.logger:
                self.logs.update({"Status": "Message handler stopped"})

    def _step_messege_handler(self) -> None:
        """
        Метод одного шага симуляции дрона

        :return: None
        :rtype: None
        """
        self.velocity_controller()
        for object_channel, simulation_object in enumerate(
            self.simulation_objects
        ):
            self.step(simulation_object, object_channel)
        # Изменено: обновляем массив последних точек с полным вектором [x, y, z, yaw]
        current_full_position = np.hstack(
            [
                self.simulation_objects[0].state[0:3],
                np.array([self.simulation_objects[0].attitude[2]]),
            ]
        )
        self.last_points = update_array(
            self.last_points, current_full_position
        )
        if self.logger:
            self.print_information()

    def _message_handler(self, *args) -> None:
        """
        Основной цикл обработки сообщений

        :return: None
        :rtype: None
        """
        last_time = time.time()
        self._pid_position_controller = PIDController(
            *self.position_pid_matrix
        )
        self._pid_velocity_controller = PIDController(
            *self.velocity_pid_matrix
        )
        while self.simulation_turn_on:
            with self._handler_lock:
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    last_time = current_time
                    self._heartbeat_send_time = current_time
                    self._step_messege_handler()
                    if self.check_attitude_flag:
                        self.attitude_write()
            time.sleep(0.01)

    def velocity_controller(self) -> None:
        """
        Метод высчитывает необходимую силу для внутренней модели self.simulation_objects для достижения таргетной скорости из t_speed.

        :return: None
        :rtype: None
        """
        current_yaw_rate = self.simulation_objects[0].attitude[5]
        current_velocity = np.hstack(
            [self.speed, np.array([current_yaw_rate])]
        )
        signal = self._pid_velocity_controller.compute_control(
            target_position=np.array(self.t_speed, dtype=np.float64),
            current_position=np.array(current_velocity, dtype=np.float64),
            dt=self.dt,
        )
        signal = np.clip(signal, -self.max_acceleration, self.max_acceleration)
        self.set_force(signal, 0)

    def position_controller(self, position_xyz: Array4) -> None:
        """
        Метод высчитывает необходимую скорость для достижения таргетной позицыы position_xyz_yaw

        :param position_xyz: Таргетная позиция дрона
        :type position_xyz: Array4
        :return: None
        :rtype: None
        """
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(
                    np.hstack(
                        [
                            position_xyz,
                        ]
                    ),
                    dtype=np.float64,
                ),
                current_position=np.hstack(
                    [
                        self.simulation_objects[0].state[0:3],
                        self.simulation_objects[0].attitude[2],
                    ]
                ),
                dt=self.dt,
            ),
            -self.max_speed,
            self.max_speed,
        )
        self.t_speed = signal

    def goto(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        accuracy: float = 5e-2,
    ) -> None:
        """
        Метод берет целевую координату и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.

        Максимальная скорость обрезается np.clip по полю self.max_speed

        :param x: координата по x
        :param y: координата по y
        :param z:  координата по z
        :param yaw:  координата по yaw
        :param accuracy: Погрешность целевой точки

        :return: None
        """
        self.threads.append(
            start_threading(self.goto_from_outside, x, y, z, yaw, accuracy)
        )

    def goto_from_outside(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        accuracy: float = 5e-2,
        wait: bool = True,
    ) -> None:
        """
        Запускает симуляцию движения дрона к заданной точке.

        :param x: Целевая координата x
        :type x: float
        :param y: Целевая координата y
        :type y: float
        :param z: Целевая координата z
        :type z: float
        :param yaw: Целевой угол рысканья
        :type yaw: float
        :param accuracy: Точность достижения цели
        :type accuracy: float
        :param wait: Ожидать завершения движения
        :type wait: bool
        :return: None
        """
        target_point = [x, y, z, yaw]
        self.point_reached = True
        with self._handler_lock:
            last_time = time.time()
            self.point_reached = False
            self._pid_position_controller = PIDController(
                *self.position_pid_matrix
            )
            self._pid_velocity_controller = PIDController(
                *self.velocity_pid_matrix
            )
            self.__goto_process = True
            while not self.point_reached and self.__goto_process:
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    self.point_reached = vector_reached(
                        target_point, self.last_points, accuracy=accuracy
                    )
                    current_full_position = np.hstack(
                        [
                            self.simulation_objects[0].state[0:3],
                            np.array([self.simulation_objects[0].attitude[2]]),
                        ]
                    )
                    self.last_points = update_array(
                        self.last_points, current_full_position
                    )
                    self.velocity_controller()
                    print("Я вызываюсь из goto")
                    self.position_controller(np.array(target_point))
                    last_time = current_time
                    for object_channel, simulation_object in enumerate(
                        self.simulation_objects
                    ):
                        self.step(simulation_object, object_channel)
                    if self.logger:
                        self.print_information()
                time.sleep(0.01)
            if self.logger:
                self.logs.update(
                    {"Регулятор положения": f"Точка {target_point} достигнута"}
                )
            self.t_speed = np.zeros(4)

    def stop(self) -> None:
        """
        Останавливает все потоки, завершает симуляцию

        :return: None
        """
        self.tracking = False
        self.speed_flag = False
        self.simulation_turn_on = False
        print("Simulation stopped")

    def borders(self) -> None:
        """
        Метод накладывает границы симуляции для дрона

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
        self.simulation_objects[0].position = np.clip(
            position, self.lower_bound, self.upper_bound
        )

    def goto_body(self, body_target: Union[Array3, Array4]) -> None:
        """
        Перемещает дрон в точку, заданную относительно системы координат, закрепленной за дроном.

        Параметр body_target задаётся относительно нуля дрона в его собственной системе координат.
        Если вектор имеет длину 3, считается, что задаётся смещение [dx, dy, dz] (без изменения yaw),
        а если длина 4 – последний элемент добавляется к текущему yaw.
        После преобразования целевая точка вычисляется в инерциальных координатах и вызывается стандартный goto.

        :param body_target: вектор смещения относительно тела дрона (например, [dx, dy, dz] или [dx, dy, dz, dyaw])
        :type body_target: Union[Array3, Array4]
        :return: None
        """
        self.__goto_process = False
        current_yaw = self.simulation_objects[0].attitude[2]
        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)
        R = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        offset_xy = R @ body_target[0:2]
        offset_z = body_target[2] if len(body_target) >= 3 else 0.0
        current_pos = self.position[0:3]
        target_xyz = current_pos + np.hstack([offset_xy, [offset_z]])
        if len(body_target) == 4:
            target_yaw = (
                self.simulation_objects[0].attitude[2] + body_target[3]
            )
        else:
            target_yaw = self.simulation_objects[0].attitude[2]
        self.goto(target_xyz[0], target_xyz[1], target_xyz[2], target_yaw)

    def set_body_velocity(self, body_vel: Array4) -> None:
        """
        Устанавливает скорость дрона, заданную в системе координат, закрепленной за дроном.

        Параметр body_vel задаёт скорость относительно тела дрона. Эта скорость преобразуется в инерциальную систему,
        с учётом текущего yaw дрона, после чего вызывается метод set_force.

        :param body_vel: скорость в системе координат дрона (например, [vx_body, vy_body, vz_body, v_yaw])
        :type body_vel: NDArray[np.float64]
        :return: None
        """
        current_yaw = self.simulation_objects[0].attitude[2]
        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)
        R = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        inertial_xy = R @ body_vel[0:2]
        inertial_z = body_vel[2] if len(body_vel) >= 3 else 0.0

        inertial_velocity = np.hstack([inertial_xy, [inertial_z]])
        velocity_command = np.hstack([inertial_velocity, [body_vel[3]]])
        self.send_speed(*velocity_command)

    def send_speed(
        self, vx: float, vy: float, vz: float, yaw_rate: float
    ) -> None:
        """
        Метод задает вектор скорости дрону. Отсылать необходимо в цикле.

        :param vx: скорость по оси x (м/с)
        :type vx: float
        :param vy: скорость по оси y (м/с)
        :type vy: float
        :param vz:  скорость по оси z (м/с)
        :type vz: float
        :param yaw_rate:  скорость поворота по оси z (рад/с)
        :type yaw_rate: float
        :return: None
        """
        self.t_speed = np.array([vx, vy, vz, yaw_rate])

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
        self.__goto_process = False
        throttle = normalize_channel(channel_1)
        yaw = normalize_channel(channel_2)
        pitch = -normalize_channel(channel_3)
        roll = normalize_channel(channel_4)

        current_yaw = self.yaw
        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)

        vx_global = pitch * cos_yaw - roll * sin_yaw
        vy_global = pitch * sin_yaw + roll * cos_yaw

        vx = vx_global * self.max_speed
        vy = vy_global * self.max_speed
        vz = throttle * self.max_speed
        vyaw = yaw * self.max_yaw_rate

        self.t_speed = np.array([vx, vy, vz, vyaw])
