import numpy as np
from numpy import cos, sin
from pymavlink import mavutil
import time
import threading


def create_connection(connection_method, ip, port):
    """
    create mavlink connection
    :return: mav_socket
    """
    mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (connection_method, ip, port))
    return mav_socket


def rot_z(vector: list | np.ndarray, angle: float | int) -> np.ndarray:
    """
    Функция вращает входные вектора вокруг оси z, заданной векторами-столбцами. Положительным вращением считается
    по часовой стрелке при направлении оси к нам.
    :param vector: Вращаемые вектор-строки, упакованные в матрицу nx3
    :param angle: угол вращения, заданный в радианах
    :return: np.ndarray
    """
    vector = np.array(vector)
    rotate_z = np.array([[cos(angle), -sin(angle), 0],
                         [sin(angle), cos(angle), 0],
                         [0, 0, 1]])
    if vector.shape == (3,) or vector.shape == (1, 3):

        rot_vector = vector.dot(rotate_z)
        return rot_vector
    elif vector.shape == (2,) or vector.shape == (1, 2):
        vector = np.hstack([vector, 0])
        rot_vector = vector.dot(rotate_z)
        return rot_vector[0:2]
    elif vector.shape == (4,) or vector.shape == (1, 4):
        rot_vector = vector[0:3].dot(rotate_z)
        return np.hstack([rot_vector, vector[3]])


class Pion:
    def __init__(self, ip: str = '10.1.100.114',
                 mavlink_port: int = 8001,
                 connection_method: str = 'udpout',
                 rotate_xyz: int | float = 0,
                 combine_system: int=0):
        self.mavlink_socket = create_connection(connection_method=connection_method,
                                                ip=ip, port=mavlink_port)
        self._heartbeat_timeout = 1
        self._mavlink_send_number = 10
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()
        self._message_handler_thread = threading.Thread(target=self._message_handler, args=(combine_system=combine_system), daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()
        self._attitude = np.array([0, 0, 0, 0, 0, 0])
        # Флаг для остановки цикла отдачи вектора скорости дрону
        self.speed_flag = True
        # Флаг для остановки сохранения координат
        self.check_attitude_flag = True
        # Список потоков
        self.threads = []
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
        self.t_speed = np.array([0, 0, 0, 0])
        self.rotate_xyz = rotate_xyz
        self.period_send_speed = 0.05
        self.period_get_attitude = 0.05
        # Информация, включающая
        # x, y, z, yaw, vx, vy, vz, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t,
        # которая складывается в матрицу (n, 12), где n - число измерений
        self.trajectory = np.zeros((11,))
        # Время создания экземпляра
        self.t0 = time.time()

    @property
    def attitude(self) -> np.ndarray:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return self._attitude

    @attitude.setter
    def attitude(self, attitude):
        self._attitude = attitude

    def arm(self):
        return self._send_command_long(command_name='ARM', command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       param1=1)

    def disarm(self):
        return self._send_command_long(command_name='DISARM', command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       param1=0)

    def takeoff(self):
        return self._send_command_long(command_name='TAKEOFF', command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def land(self):
        return self._send_command_long(command_name='LAND', command=mavutil.mavlink.MAV_CMD_NAV_LAND)

    def goto(self, x: float | int,
             y: float | int,
             z: float | int,
             yaw: float | int = 0) -> None:
        """ Flight to point in the current navigation system's coordinate frame """
        # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        mask = 0b0000_10_0_111_111_000
        # ENU coordinates to NED coordinates
        x, y, z = y, x, -z
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask, x=x, y=y, z=z, yaw=yaw)

    def send_speed(self, vx: float | int,
                   vy: float | int,
                   vz: float | int,
                   yaw_rate: float | int) -> None:
        """
        Функция задает вектор скорости дрону. Отсылать необходимо в цикле.
        :param vx: скорость по оси x
        :param vy: скорость по оси y
        :param vz:  скорость по оси z
        :param yaw_rate:  скорость поворота по оси z
        :return: None
        """
        # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        mask = 0b0000_01_0_111_000_111
        # ENU coordinates to NED coordinates
        vx, vy, vz = vy, vx, -vz
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def _send_position_target_local_ned(self, coordinate_system, mask=0b0000_11_0_111_111_111, x=0, y=0,
                                        z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0,
                                        target_system=None, target_component=None) -> None:
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        for confirm in range(self._mavlink_send_number):
            self.mavlink_socket.mav.set_position_target_local_ned_send(0, target_system, target_component,
                                                                       coordinate_system,
                                                                       mask, x, y, z, vx, vy, vz, afx, afy, afz,
                                                                       yaw, yaw_rate)

    def _send_command_long(self, command_name, command, param1: float = 0, param2: float = 0, param3: float = 0,
                           param4: float = 0, param5: float = 0, param6: float = 0, param7: float = 0,
                           target_system=None, target_component=None) -> None:
        print(f"_send_command_long --> {command_name}")
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        confirm = 0
        while True:
            self.mavlink_socket.mav.command_long_send(target_system, target_component, command, confirm,
                                                      param1, param2, param3, param4, param5, param6, param7)
            confirm += 1
            if confirm >= self._mavlink_send_number:
                break

    def _send_heartbeat(self) -> None:
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self._heartbeat_send_time = time.time()

    def _message_handler(self, combine_system: int=0) -> None:
        """
        Функция поддерживает отправку хартбитов на дрон, а также пишет актуальные координаты в attitude
        Если combine_system = 0, то данные читаются только с локуса, если combine_system = 1, то данные читаются и с локуса и с оптики, 
        если combine_system = 2, то данные читаются только с оптики
        """
        if combine_system == 0:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                    self._send_heartbeat()
                msg = self.mavlink_socket.recv_msg()
                if msg is not None:
                    if msg.get_type() == "LOCAL_POSITION_NED" and msg._header.srcComponent == 1:
                        self.attitude = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
        elif combine_system == 1:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                    self._send_heartbeat()
                msg = self.mavlink_socket.recv_msg()
                if msg is not None:
                    if msg.get_type() == "LOCAL_POSITION_NED":
                        self.attitude = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
        elif combine_system == 2:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                    self._send_heartbeat()
                msg = self.mavlink_socket.recv_msg()
                if msg is not None:
                    if msg.get_type() == "LOCAL_POSITION_NED" and msg._header.srcComponent == 26:
                        self.attitude = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])


    def v_while(self, ampl: float | int = 1) -> None:
        """
        Функция задает цикл while на отправку вектора скорости в body с периодом period_send_v
        :param ampl: Амплитуда усиления вектора скорости
        :type ampl: float | int
        :return: None
        """
        if self.rotate_xyz == 0:
            while self.speed_flag:
                t_speed = self.t_speed * ampl
                self.send_speed(t_speed[0], t_speed[1], t_speed[2], t_speed[3])
                time.sleep(self.period_send_speed)
        else:
            v = ampl * rot_z(self.t_speed[0:3], self.rotate_xyz)
            while self.speed_flag:
                self.send_speed(v[0], v[1], v[2], self.t_speed[3])
                time.sleep(self.period_send_speed)

    def set_v(self, ampl: float | int = 1) -> None:
        """
        Создает поток, который вызывает функцию v_while() для параллельной отправки вектора скорости
        :param ampl: Амплитуда усиления вектора скорости
        :type ampl: float | int
        :return: None
        """
        self.speed_flag = True
        self.threads.append(threading.Thread(target=self.v_while, args=[ampl]))
        self.threads[-1].start()

    def reboot_board(self) -> None:
        """
        Функция для перезагрузки дрона
        :return: None
        """
        self._send_command_long(command_name='REBOOT_BOARD',
                                command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                target_component=1,
                                param1=1)

    def set_attitude_check(self) -> None:
        """
        Функция запускает поток отслеживания координат
        :return: None
        """
        self.threads.append(threading.Thread(target=self.attitude_write))
        self.threads[-1].start()

    def attitude_write(self) -> None:
        """
        Функция для записи траектории в numpy массив
        :return:
        """
        while self.check_attitude_flag:
            t = time.time() - self.t0
            stack = np.hstack([self.attitude, self.t_speed, [t]])
            self.trajectory = np.vstack([self.trajectory, stack])
            time.sleep(self.period_get_attitude)

    def save_data(self, file_name: str = 'data.npy', path: str = '') -> None:
        """
        Функция для сохранения траектории в файл
        columns=['x', 'y', 'z', 'yaw', 'Vx', 'Vy', 'Vz', 'Vy_yaw', 'vxc', 'vyc', 'vzc', 'v_yaw_c', 't']
        :param file_name: название файла
        :param path: путь сохранения
        :return: None
        """
        self.speed_flag = False
        np.save(f'{path}{file_name}', self.trajectory[1:])
