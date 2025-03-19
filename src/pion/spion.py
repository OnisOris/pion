from .simulator import Simulator, Point
from .pio import DroneBase
from .functions import vector_reached, update_array
from pion.cython_pid import PIDController  # Cython-версия PIDController
from typing import Union, Optional, Any, Annotated
from .annotation import Array6, Array4, Array3, Array2
import numpy as np
from numpy.typing import NDArray
import time
import threading


class Spion(Simulator, DroneBase):
    """
    Класс симулятор, повторяющий действия Pion в симуляции математической модели точки
    """
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 connection_method: str = 'udpout',
                 position: Optional[Union[Array6, Array4]] = None,
                 attitude: Optional[Union[Array6, Array4]] = None,
                 combine_system: int = 0,
                 count_of_checking_points: int = 20,
                 name: str = 'simulator',
                 mass: float = 0.3,
                 dt: float = 0.1,
                 logger: bool = False,
                 checking_components: bool = True,
                 accuracy: float = 2e-5,
                 max_speed: float = 2.,
                 start_message_handler_from_init: bool = True,
                 dimension: int = 3) -> None:
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

        :param dimension: Размерность дрона, возможные значения: 2, 3
        :type dimension: int
        """
        DroneBase.__init__(self,
                           ip=ip,
                           mavlink_port=mavlink_port,
                           name=name,
                           mass=mass,
                           dimension=dimension,
                           position=position,
                           attitude=attitude,
                           count_of_checking_points=count_of_checking_points,
                           logger=logger,
                           checking_components=checking_components,
                           accuracy=accuracy,
                           dt=dt,
                           max_speed=max_speed)
        # Создание объекта Point3D
        self.simulation_objects: NDArray[Any] = np.array([Point(mass, self._position[0:self.dimension],
                                                  self._position[self.dimension:self.dimension * 2])])
        self.connection_method: str = connection_method
        self.combine_system: int = combine_system
        self.count_of_checking_points: int = count_of_checking_points

        self.position_pid_matrix:  Annotated[NDArray[Any], (3, self.dimension)] = np.array([
            [1.0] * self.dimension,
            [0.0] * self.dimension,
            [0.0] * self.dimension
        ], dtype=np.float64)

        self.velocity_pid_matrix: Annotated[NDArray[Any], (3, self.dimension)] = np.array([
            [3.0] * self.dimension,
            [0.0] * self.dimension,
            [0.1] * self.dimension
        ], dtype=np.float64)
        Simulator.__init__(self, self.simulation_objects, dt=dt, dimension=self.dimension)
        self.max_speed: float = 2.
        # Период отправления следующего вектора скорости
        self.period_send_speed: float = 0.05
        self.speed_flag: bool = True
        self._pid_velocity_controller: Optional[PIDController] = None
        self._heartbeat_send_time: float = time.time()
        self._heartbeat_timeout: float = 3.
        # Границы симуляции
        self.lower_bound: Array3 = np.array([-5.5, -5.5, 0])
        self.upper_bound: Array3 = np.array([5.5, 5.5, 4])
        self.point_reached: bool = False
        self._message_thread: Optional[threading.Thread] = None  # Поток для _message_handler
        self.last_points: Annotated[NDArray[Any], (count_of_checking_points, self.dimension)] = np.zeros((count_of_checking_points, self.dimension))
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

        :return: Скорость [vx, vy, vz]
        :rtype: Union[Array2, Array3]
        """
        return self.simulation_objects[0].speed

    def takeoff(self) -> None:
        """
        Функция взлета дрона

        :return: None
        :rtype: None
        """
        super().takeoff()
        self.goto(self.position[0], self.position[1], 1.5, 0)


    def land(self) -> None:
        """
        Функция посадки дрона
        :return: None
        :rtype: None
        """
        super().land()
        self.goto(self.position[0], self.position[1], 0, 0)


    def start_message_handler(self) -> None:
        """
        Запуск потока _message_handler
        :return: None
        :rtype: None
        """
        if not self.simulation_turn_on:
            self.simulation_turn_on = True
            self._message_thread = threading.Thread(target=self._message_handler)
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
        Функция одного шага симуляции дрона
        :return: None
        :rtype: None
        """
        self.velocity_controller()
        for object_channel, simulation_object in enumerate(self.simulation_objects):
            self.step(simulation_object, object_channel)
        self.last_points = update_array(self.last_points, self.position[0:self.dimension])
        if self.logger:
            self.print_information()


    def _message_handler(self, *args) -> None:
        """
        Основной цикл обработки сообщений
        :return: None
        :rtype: None
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

    def velocity_controller(self) -> None:
        """
        Функция высчитывает необходимую силу для внутренней модели self.simulation_objects для
        достижения таргетной скорости из t_speed

        :return: None
        :rtype: None
        """
        signal = self._pid_velocity_controller.compute_control(
            target_position=np.array(self.t_speed[0:self.dimension], dtype=np.float64),
            current_position=np.array(self.simulation_objects[0].speed, dtype=np.float64),
            dt=self.dt)
        self.set_force(signal, 0)

    def position_controller(self,
                            position_xyz: Array3) -> None:
        """
        Функция высчитывает необходимую скорость для достижения таргетной позицыы position_xyz
        :param position_xyz: Таргетная позиция дрона
        :type position_xyz: Array3
        :return: None
        :rtype: None
        """
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(position_xyz, dtype=np.float64),
                current_position=self.simulation_objects[0].position,
                dt=self.dt),
            -self.max_speed,
            self.max_speed)
        self.t_speed = np.hstack([signal, np.array([0]*(4-self.dimension))])

    def goto(self,
             x: float,
             y: float,
             z: float,
             yaw: float,
             accuracy: Optional[float] = None) -> None:
        """
        Функция берет целевую координату и вычисляет необходимые скорости для достижения целевой позиции, посылая их в
        управление t_speed.
        Максимальная скорость обрезается np.clip по полю self.max_speed
        :param x: координата по x
        :type x: float
        :param y: координата по y
        :type: float
        :param z:  координата по z (не используется, если self.dimension = 2)
        :type: float
        :param yaw:  координата по yaw
        :type: float
        :param accuracy: Погрешность целевой точки 
        :type: float
        :return: None
        """
        if self.dimension == 2:
            target_point = [x, y]
        else:
            target_point = [x, y, z]
        if accuracy is None:
            accuracy = self.accuracy
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
                        self.print_information()
                time.sleep(0.01)  # Даем CPU немного отдохнуть
            if self.logger:
                self.logs.update({"Регулятор положения": f"Точка {target_point} достигнута"})
            self.t_speed = np.zeros(self.dimension + 1)

    def goto_from_outside(self,
                          x: float,
                          y: float,
                          z: float,
                          yaw: float,
                          accuracy: float = 5e-2) -> None:
        """
        Функция симулятор оригинальной функции в Pion, полностью повторяет функционал goto в данном классе
        :param x: координата по x
        :type x: float
        :param y: координата по y
        :type: float
        :param z:  координата по z
        :type: float
        :param yaw:  координата по yaw
        :type: float
        :param accuracy: Погрешность целевой точки
        :type: float
        :return: None
        """
        self.goto(x, y, z, yaw, accuracy)

    def stop(self) -> None:
        """
        Останавливает все потоки, завершает симуляцию
        :return: None
        :rtype: None
        """
        self.tracking = False
        self.speed_flag = False
        self.simulation_turn_on = False
        print("Simulation stopped")

    def borders(self) -> None:
        """
        Функция накладывает границы симуляции для дрона
        :return: None
        :rtype: None
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
