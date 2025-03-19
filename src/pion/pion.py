import select
import threading
import time
from numpy.typing import NDArray
from typing import Tuple, Union, Optional, Annotated, Any
import numpy as np
from pion.cython_pid import PIDController
from pymavlink.dialects.v10.all import MAVLink_message
from .annotation import Array6, Array4, Array3, Array2
from .functions import create_connection, start_threading, vector_reached, scalar_reached, update_array, update_vector
from pymavlink import mavutil
from .pio import DroneBase


class Pion(DroneBase):
    """
    Класс Pion предназначен для управления дроном через протокол MAVLink. Он включает функционал для инициализации
    соединения, отправки команд дрону, обработки сообщений, и управления движением. Pion также поддерживает
    многопоточность для выполнения различных параллельных задач
    """
    def __init__(self,
                 ip: str = '10.1.100.114',
                 mavlink_port: int = 5656,
                 connection_method: str = 'udpout',
                 position: Optional[Union[Array6, Array4]] = None,
                 attitude: Optional[Union[Array6, Array4]] = None,
                 combine_system: int = 0,
                 count_of_checking_points: int = 20,
                 name: str = "Pion",
                 mass: float = 0.3,
                 dt: float = 0.,
                 logger: bool = False,
                 start_message_handler_from_init: bool = True,
                 checking_components: bool = True,
                 accuracy: float = 5e-2,
                 max_speed: float = 2.,
                 dimension: int = 3):
        """
        Инициализация класса Pion, устанавливающего MAVLink соединение с дроном 
        и управляющего взаимодействием по передаче и приему данных.

        :param ip: IP-адрес для подключения к дрону
        :type ip: str
        
        :param mavlink_port: Порт для MAVLink соединения.
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

        :param start_message_handler_from_init: Старт message handler при создании объекта
        :type start_message_handler_from_init: bool

        :param checking_components: Параметр для проверки номеров компонентов. Отключается в сторонних симуляторах
         для избежание ошибок.
        :type checking_components: bool

        :param accuracy: Максимальное отклонение от целевой позиции для функции goto_from_outside
        :type accuracy: float

        :param max_speed: Максимальная скорость дрона в режиме управления по скорости
        :type max_speed: float

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
        # Флаг для остановки цикла отдачи вектора скорости дрону
        self.speed_flag: bool = True
        # Флаг для остановки отдачи управляющих сигналов rc channels
        self.rc_flag: bool = False
        # Флаг для остановки цикла в _message_handler
        self.message_handler_flag: bool = True
        # Последнее сообщение из _message_handler()
        self._msg: Optional["MAVLink_message"] = None

        self.mavlink_socket: mavutil.mavfile = create_connection(connection_method=connection_method,
                                                address=ip,
                                                port_or_baudrate=mavlink_port)
        self._heartbeat_timeout: float = 1.
        self._mavlink_send_number: int = 10
        self.__is_socket_open: threading.Event = threading.Event()
        self.__is_socket_open.set()
        # Список потоков
        self.threads: list = []
        # Период отправления следующего вектора скорости
        self.period_send_speed: float = 0.05
        # Период отправления rc каналов
        self.period_send_rc: float = 0.05
        # Период приема всех сообщений с дрона
        self.period_message_handler: float = dt
        self.connection_lost: bool = False
        self.max_speed: float = 1.
        # Используется для хранения последних count_of_checking_points данных в виде [x, y, z] для верификации достижения таргетной точки
        self.last_points: Annotated[NDArray[Any], (count_of_checking_points,)] = np.zeros((count_of_checking_points, self.dimension))
        # Используется для хранения последних 14 значений yaw в матрице для верификации достижения таргетного угла по z
        self.last_angles: Annotated[NDArray[Any], (14,)]  = np.zeros(14) 
        if start_message_handler_from_init:
            self._message_handler_thread: threading.Thread = threading.Thread(target=self._message_handler,
                                                                              args=(combine_system,))
            self._message_handler_thread.start()
        self.position_pid_matrix: np.ndarray = np.array([
            [0.5] * self.dimension,
            [0.0] * self.dimension,
            [0.7] * self.dimension
        ], dtype=np.float64)
        self.yaw_pid_matrix: np.ndarray = np.array([
            [1] * 1,
            [0] * 1,
            [1] * 1
        ], dtype=np.float64)
        self.set_v_check_flag: bool = False
        self.set_rc_check_flag: bool = False
        self.target_point: np.ndarray = np.array([0, 0, 2, 0])
        self.tracking: bool = False


    @property
    def speed(self) -> Union[Array2, Array3]:
        """
        Функция вернет скорость [vx, vy, vz]

        :return: Union[Array2, Array3]
        """
        return self._position[self.dimension:self.dimension * 2]

    def arm(self) -> None:
        """
        Включает двигатели

        :return: None
        """
        super().arm()
        self._send_command_long(command_name='ARM',
                                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                param1=1,
                                mavlink_send_number=self._mavlink_send_number)

    def disarm(self) -> None:
        """
        Отключает двигатели

        :return: None
        """
        super().disarm()
        self._send_command_long(command_name='DISARM',
                                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                param1=0,
                                mavlink_send_number=self._mavlink_send_number)

    def takeoff(self) -> None:
        """
        Взлет дрона

        :return: None
        """
        super().takeoff()
        self._send_command_long(command_name='TAKEOFF',
                                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                mavlink_send_number=self._mavlink_send_number)

    def land(self) -> None:
        """
        Посадка дрона
        
        :return: None
        """
        super().land()
        self._send_command_long(command_name='LAND',
                                command=mavutil.mavlink.MAV_CMD_NAV_LAND,
                                mavlink_send_number=self._mavlink_send_number)

    def goto(self,
             x: Union[float, int],
             y: Union[float, int],
             z: Union[float, int],
             yaw: Union[float, int]) -> None:
        """
        Полет к указанной точке в текущей системе координат навигации.

        :param x: Координата по оси X в ENU (East-North-Up) системе координат.
        :type x: float | int
        :param y: Координата по оси Y в ENU (East-North-Up) системе координат.
        :type y: float | int
        :param z: Координата по оси Z (высота) в ENU (East-North-Up) системе координат.
        :type z: float | int
        :param yaw: Угол курса, на который должен повернуться дрон. По умолчанию 0.
        :type yaw: float | int, Optional
        :return: None

        :note: Координаты задаются в ENU (East-North-Up) системе координат, но будут автоматически преобразованы 
            в NED (North-East-Down).
        """
        self.tracking = False
        mask = 0b0000_10_0_111_111_000
        x, y, z = y, x, -z
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask,
                                             x=x,
                                             y=y,
                                             z=z,
                                             yaw=yaw,
                                             mavlink_send_number=10)

    def start_track_point(self) -> None:
        """
        Старт режима слежения за точкой дроном. Целевая точка меняется в поле self.target_point

        :return: None
        :rtype: None
        """
        start_threading(self.point_tracking)
        self.threads.append(threading.Thread(target=self.point_tracking))

    def point_tracking(self) -> None:
        """
        Функция слежения за точкой. Целевая точка меняется в поле self.target_point.

        :return: None
        :rtype: None
        """
        self.set_v()
        self.goto_yaw(0)
        self._pid_position_controller = PIDController(*self.position_pid_matrix) 
        self.point_reached = False
        last_time = time.time()
        time.sleep(self.period_send_speed)
        self.tracking = True
        print(1)
        while self.tracking:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.position_controller(self.target_point[0:3], dt)
            time.sleep(self.period_send_speed)
        self.t_speed = np.zeros(4)

    def goto_from_outside(self,
                          x: float,
                          y: float,
                          z: float,
                          yaw: float,
                          accuracy: Optional[float] = None) -> None:
        """
        Функция берет целевую координату и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.
        Для использования необходимо включить цикл :py:meth:`Pion.v_while` для посылки вектора скорости дрону.
        Максимальная скорость обрезается np.clip по полю self.max_speed
        
        :param x: координата по x
        :type x: float
        :param y: координата по y
        :type y: float
        :param z: координата по z
        :type z: float
        :param yaw: координата по yaw
        :type yaw: float
        :param accuracy: Погрешность целевой точки
        :type accuracy: Optional[float]
        :return: None
        """
        self.tracking = False
        self.set_v()
        self.goto_yaw(yaw)
        if self.dimension == 2:
            target_point = np.array([x, y])
        else:
            target_point = np.array([x, y, z])
        if accuracy is None:
            accuracy = self.accuracy
        self._pid_position_controller = PIDController(*self.position_pid_matrix) 
        self.point_reached = False
        last_time = time.time()
        time.sleep(self.period_send_speed)
        while not self.point_reached:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.point_reached = vector_reached(target_point,
                                                self.last_points,
                                                accuracy=accuracy)
            self.position_controller(target_point, dt)
            time.sleep(self.period_send_speed)
        self.t_speed = np.zeros(4)

    def goto_yaw(self,
                 yaw: float = 0.,
                 accuracy: float = 0.057) -> None:
        """
        Функция берет целевую координату по yaw и вычисляет необходимые скорости для достижения целевой позиции, посылая их в управление t_speed.
        Для использования необходимо включить цикл :py:meth:`Pion.v_while` для посылки вектора скорости дрону.
        Максимальная скорость обрезается np.clip по полю self.max_speed

        :param yaw:  координата по yaw (радианы)
        :type yaw: float
        :param accuracy: Погрешность целевой точки
        :type accuracy: float
        :return: None
        """
        self.tracking = False
        self.set_v()
        pid_controller = PIDController(*self.yaw_pid_matrix)
        self.point_reached = False
        last_time = time.time()
        time.sleep(self.period_send_speed)
        while not self.point_reached:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.point_reached = scalar_reached(yaw, self.last_angles, accuracy=accuracy)
            signal = -pid_controller.compute_control(np.array([yaw], dtype=np.float64),
                                                     np.array([self.yaw],
                                                              dtype=np.float64),
                                                     dt=dt)[0]
            self.t_speed = np.array([*np.zeros(3), np.clip(signal,
                                                           -self.max_speed, self.max_speed)])
            time.sleep(self.period_send_speed)
        self.t_speed = np.zeros(4)

    def send_speed(self,
                   vx: float,
                   vy: float,
                   vz: float,
                   yaw_rate: float) -> None:
        """
        Функция задает вектор скорости дрону. Отсылать необходимо в цикле.

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
        # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        mask = 0b0000_01_0_111_000_111
        # ENU coordinates to NED coordinates
        vx, vy, vz = vy, vx, -vz
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask,
                                             vx=vx,
                                             vy=vy,
                                             vz=vz,
                                             yaw_rate=yaw_rate,
                                             mavlink_send_number=1)

    def send_rc_channels(self,
                         channel_1: int = 0xFF,
                         channel_2: int = 0xFF,
                         channel_3: int = 0xFF, 
                         channel_4: int = 0xFF) -> None:
        """
        Функция отправляет управляющие сигналы RC-каналов дрону. Отсылать необходимо в цикле.

        :param channel_1: высота (throttle)
        :type channel_1: int
        :param channel_2: угол курса (yaw)
        :type channel_2: int
        :param channel_3: движение влево/вправо (roll)
        :type channel_3: int
        :param channel_4: движение вперед/назад (pitch)
        :type channel_4: int
        :return: None
        """
        # Фиксированные значения для каналов 5-8
        channel_5 = 0xFF
        channel_6 = 0xFF
        channel_7 = 0xFF
        channel_8 = 0xFF

        self.mavlink_socket.mav.rc_channels_override_send(self.mavlink_socket.target_system,
                                                          self.mavlink_socket.target_component, channel_1,
                                                          channel_2, channel_3, channel_4, channel_5, channel_6,
                                                          channel_7, channel_8)

    def _send_position_target_local_ned(self, coordinate_system,
                                        mask=0b0000_11_0_111_111_111,
                                        x: Union[float, int] = 0,
                                        y: Union[float, int] = 0,
                                        z: Union[float, int] = 0,
                                        vx: Union[float, int] = 0,
                                        vy: Union[float, int] = 0,
                                        vz: Union[float, int] = 0,
                                        afx: Union[float, int] = 0,
                                        afy: Union[float, int] = 0,
                                        afz: Union[float, int] = 0,
                                        yaw: Union[float, int] = 0,
                                        yaw_rate: Union[float, int] = 0,
                                        target_system=None,
                                        target_component=None,
                                        mavlink_send_number=1) -> None:
        """
        Функция отправляет команду MAVLink для установки целевой позиции
        или скорости в локальной системе координат NED (North, East, Down). 
        Параметры включают систему координат, маску для указания активных полей,
        координаты (x, y, z), скорости (vx, vy, vz), ускорения и скорость поворота
        по оси yaw

        :param coordinate_system: Система координат (например, NED).
        :type coordinate_system: int

        :param int mask: Битовая маска для указания, какие измерения будут проигнорированы в сообщении MAVLink. 
        Соответствует спецификации MAVLink `POSITION_TARGET_TYPEMASK`:
        - Биты 0-2: Игнорировать позицию (x, y, z)
        - Биты 3-5: Игнорировать скорость (vx, vy, vz)
        - Биты 6-8: Игнорировать ускорение или силу (afx, afy, afz)
        - Бит 9: Игнорировать курс (yaw)
        - Бит 10: Игнорировать скорость изменения курса (yaw_rate)
        Пример: 0b0000_11_0_111_111_111 означает игнорирование скорости, ускорения, курса и скорости изменения курса.

        :param x: Координата x.
        :type x: Union[float, int]
        :param y: Координата y.
        :type y: Union[float, int]
        :param z: Координата z.
        :type z: Union[float, int]
        :param vx: Скорость по оси x.
        :type vx: Union[float, int]
        :param vy: Скорость по оси y.
        :type vy: Union[float, int]
        :param vz: Скорость по оси z.
        :type vz: Union[float, int]
        :param afx: Ускорение по оси x.
        :type afx: Union[float, int]
        :param afy: Ускорение по оси y.
        :type afy: Union[float, int]
        :param afz: Ускорение по оси z.
        :type afz: Union[float, int]
        :param yaw: Угол курса.
        :type yaw: Union[float, int]
        :param yaw_rate: Скорость изменения курса.
        :type yaw_rate: Union[float, int]
        :param target_system: Идентификатор целевой системы.
        :type target_system: int, optional
        :param target_component: Идентификатор целевого компонента.
        :type target_component: int, optional
        :param mavlink_send_number: Количество отправок команды.
        :type mavlink_send_number: int
        :return: None        
        """
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        for _ in range(mavlink_send_number):
            self.mavlink_socket.mav.set_position_target_local_ned_send(0,
                                                                       target_system,
                                                                       target_component,
                                                                       coordinate_system,
                                                                       mask,
                                                                       x,
                                                                       y,
                                                                       z,
                                                                       vx,
                                                                       vy,
                                                                       vz,
                                                                       afx,
                                                                       afy,
                                                                       afz,
                                                                       yaw,
                                                                       yaw_rate)

    def _send_command_long(self,
                           command_name: str,
                           command: int,
                           param1: Union[float, int] = 0,
                           param2: Union[float, int] = 0,
                           param3: Union[float, int] = 0,
                           param4: Union[float, int] = 0,
                           param5: Union[float, int] = 0,
                           param6: Union[float, int] = 0,
                           param7: Union[float, int] = 0,
                           target_system=None,
                           target_component=None,
                           mavlink_send_number: int = 1) -> None:
        """
        Отправляет команду типа COMMAND_LONG через MAVLink
        :param command_name: Имя команды для логирования.
        :type command_name: str
        :param command: Команда MAVLink.
        :type command: int
        :param param1: Параметр 1 команды.
        :type param1: Union[float, int]
        :param param2: Параметр 2 команды.
        :type param2: Union[float, int]
        :param param3: Параметр 3 команды.
        :type param3: Union[float, int]
        :param param4: Параметр 4 команды.
        :type param4: Union[float, int]
        :param param5: Параметр 5 команды.
        :type param5: Union[float, int]
        :param param6: Параметр 6 команды.
        :type param6: Union[float, int]
        :param param7: Параметр 7 команды.
        :type param7: Union[float, int]
        :param target_system: Идентификатор целевой системы.
        :type target_system: int, optional
        :param target_component: Идентификатор целевого компонента.
        :type target_component: int, optional
        :param mavlink_send_number: Количество отправок команды.
        :type mavlink_send_number: int
        :return: None
        """
        if self.logger:
            print(f"_send_command_long --> {command_name}")
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        confirm = 0
        while True:
            self.mavlink_socket.mav.command_long_send(target_system,
                                                      target_component,
                                                      command,
                                                      confirm,
                                                      param1,
                                                      param2,
                                                      param3,
                                                      param4,
                                                      param5,
                                                      param6,
                                                      param7)
            confirm += 1
            if confirm >= mavlink_send_number:
                break

    def _send_heartbeat(self) -> None:
        """
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном

        :return: None
        """
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               0,
                                               0,
                                               0)
        self._heartbeat_send_time = time.time()

    def _message_handler(self,
                         combine_system: int = 0) -> None:
        """   
        Обрабатывает сообщения от дрона и отправляет heartbeat, обновляя координаты дрона

        :param combine_system: Определяет, с каких источников будут считываться данные:
                                0 — только локус, 1 — локус и оптика, 2 — только оптика.
        :type combine_system: int
        :return: None
        """

        # Определяем источник данных на основе combine_system
        src_component_map = {
            0: 1,  # Только локус
            1: None,  # Локус и оптика (неважно, откуда приходит)
            2: 26  # Только оптика
        }
        src_component = src_component_map.get(combine_system)

        while self.message_handler_flag:
            with self._handler_lock:  # Захватываем управление
                if not self.__is_socket_open.is_set():
                    break
                self.heartbeat()
                # Проверка, доступно ли новое сообщение для чтения
                rlist, _, _ = select.select([self.mavlink_socket.port.fileno()], [], [], self.period_message_handler)
                if rlist:
                    self._msg = self.mavlink_socket.recv_msg()
                    if self._msg is not None:
                        self._process_message(self._msg, src_component)
                if self.check_attitude_flag:
                    self.attitude_write()
                if self.logger:
                    self.print_information()
                time.sleep(self.period_message_handler)

    def _process_message(self,
                         msg,
                         src_component: Optional[int] = None) -> None:
        """
        Обрабатывает одно сообщение и обновляет данные (позиция, ориентация, батарея)

        :param msg: Сообщение MAVLink
        :param src_component: Источник данных, по которому фильтруется сообщение.
        :return: None
        """
        # Проверяем источник компонента, если задан
        if self.checking_components:
            if src_component is not None and msg._header.srcComponent != src_component:
                return
        if msg.get_type() == "LOCAL_POSITION_NED":
            if self.dimension == 3:
                self.position = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
            else:
                self.position = np.array([msg.x, msg.y, msg.vx, msg.vy])
            self.last_points = update_array(self.last_points, self.position[0:self.dimension])
        elif msg.get_type() == "ATTITUDE":
            self.attitude = np.array([msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed])
            self.last_angles = update_vector(self.last_angles, self.yaw)
        elif msg.get_type() == "BATTERY_STATUS":
            self.battery_voltage = msg.voltages[0] / 100

    def rc_while(self) -> None:
        """
        Функция задает цикл while на отправку управляющих сигналов rc каналов с периодом period_send_rc

        :return: None
        """
        self.set_rc_check_flag = True
        while self.rc_flag:
            self.send_rc_channels()
            time.sleep(self.period_send_rc)
        self.set_rc_check_flag = False

    def set_rc(self) -> None:
        """
        Создает поток, который вызывает функцию rc_while() для параллельной отправки управляющего сигнала rc channels

        :return:
        """
        if not self.set_rc_check_flag and not self.set_v_check_flag:
            self.rc_flag = True
            self.threads.append(threading.Thread(target=self.rc_while))
            self.threads[-1].start()

    def v_while(self) -> None:
        """
        Функция задает цикл while на отправку вектора скорости в body с периодом period_send_v

        :return: None
        """
        self.set_v_check_flag = True
        while self.speed_flag:
            with self._speed_control_lock:  # Захватываем управление
                t_speed = self.t_speed
                self.send_speed(*t_speed)
                time.sleep(self.period_send_speed)
        self.set_v_check_flag = False

    def set_v(self) -> None:
        """
        Создает поток, который вызывает функцию :py:meth:`Pion.v_while` для параллельной отправки вектора скорости

        :return: None
        """
        if not self.set_v_check_flag:
            self.speed_flag = True
            self.threads.append(threading.Thread(target=self.v_while))
            self.threads[-1].start()

    def reboot_board(self) -> None:
        """
        Функция для перезагрузки дрона

        :return: None
        """
        self._send_command_long(command_name='REBOOT_BOARD',
                                command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                target_component=1,
                                param1=1,
                                mavlink_send_number=self._mavlink_send_number)

    def stop(self) -> None:
        """
        Останавливает все потоки внутри приложения

        :return: None
        """
        self.speed_flag = False
        self.rc_flag = False
        self.check_attitude_flag = False
        self.message_handler_flag = False

    def led_control(self,
                    led_id=255,
                    r=0,
                    g=0,
                    b=0) -> None:
        """
        Управление светодиодами на дроне

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
        if led_id not in [255, 0, 1, 2, 3]:
            raise ValueError(
                f"Argument 'led_id' must have one of the following values: 0, 1, 2, 3, 255. But your value is {led_id}.")
        if r < 0 or r > 255 or g < 0 or g > 255 or b < 0 or b > 255:
            raise ValueError(
                f"Arguments 'r', 'g', 'b' must have value in [0, 255]. But your values is r={r}, g={g}, b={b}.")
        self._send_command_long(command_name='LED', command=mavutil.mavlink.MAV_CMD_USER_1,
                                param1=led_id, param2=r, param3=g, param4=b)

    def led_custom(self,
                   mode: int = 1,
                   timer: int = 0,
                   color1: Tuple[int, int, int] = (0, 0, 0),
                   color2: Tuple[int, int, int] = (0, 0, 0)) -> None:
        """
        Управляет светодиодами устройства с Raspberry Pi, задавая два цвета и режим работы.

        Цвета передаются в виде кортежей (R, G, B), где каждое значение находится в диапазоне [0, 255].
        Цветовые параметры кодируются в 24-битные числа и передаются в команду MAVLink.

        :param mode: Режим работы светодиодов
        :type mode: int
        :param timer: Время работы режима (например, длительность мигания)
        :type timer: int
        :param color1: Первый цвет в формате (R, G, B)
        :type color1: Tuple[int, int, int]
        :param color2: Второй цвет в формате (R, G, B)
        :type color2: Tuple[int, int, int]
        :return: None
        """
        param2 = (((color1[0] << 8) | color1[1]) << 8) | color1[2]
        param3 = (((color2[0] << 8) | color2[1]) << 8) | color2[2]
        param5 = mode
        param6 = timer
        return self._send_command_long('RPi_LED', mavutil.mavlink.MAV_CMD_USER_3, param2=param2, param3=param3,
                                       param5=param5, param6=param6, target_system=0, target_component=0)

    def poweroff(self) -> None:
        """
        Функция отправляет команду на выключение Raspberry Pi.
    
        :return: None
        """
        return self._send_command_long(command_name='RPi_POWEROFF',
                                   command=mavutil.mavlink.MAV_CMD_USER_2)


