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
                 rotate_xyz: int | float = 0):
        self.mavlink_socket = create_connection(connection_method=connection_method,
                                                ip=ip, port=mavlink_port)
        self._heartbeat_timeout = 1
        self._mavlink_send_number = 10
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()
        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()
        self._attitude = np.array([0, 0, 0, 0, 0, 0])
        # Флаг для остановки цикла отдачи вектора скорости дрону
        self.speed_flag = True
        # Список потоков
        self.threads = []
        # Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
        self.t_speed = np.array([0, 0, 0, 0])
        self.rotate_xyz = rotate_xyz
        self.period_send_speed = 0.05

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

    def goto(self, x, y, z, yaw) -> None:
        """ Flight to point in the current navigation system's coordinate frame """
        # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        mask = 0b0000_10_0_111_111_000
        # ENU coordinates to NED coordinates
        x, y, z = y, x, -z
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask, x=x, y=y, z=z, yaw=yaw)

    def send_speed(self, vx, vy, vz, yaw_rate) -> None:
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

    def _message_handler(self) -> None:
        while True:
            if not self.__is_socket_open.is_set():
                break
            if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                self._send_heartbeat()
            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                if msg.get_type() == "LOCAL_POSITION_NED":
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
