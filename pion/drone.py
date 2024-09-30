import numpy as np
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


class Pion:
    def __init__(self, ip: str = '10.1.100.114',
                 mavlink_port: int = 8001,
                 connection_method: str = 'udpout',
                 combine_system: int=0):
        # Последнее сообщение из _message_handler()
        self._msg = None
        # Напряжение батареи дрона (Вольты)
        self.battery_voltage = None 
        self.mavlink_socket = create_connection(connection_method=connection_method,
                                                ip=ip, port=mavlink_port)
        self._heartbeat_timeout = 1
        self._mavlink_send_number = 10
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()
        self._message_handler_thread = threading.Thread(target=self._message_handler, args=(combine_system,), daemon=True)
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
        self.period_send_speed = 0.05
        self.period_get_attitude = 0.05
        # Информация, включающая
        # x, y, z, yaw, vx, vy, vz, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t,
        # которая складывается в матрицу (n, 12), где n - число измерений
        self.trajectory = np.zeros((11,))
        # Время создания экземпляра
        self.t0 = time.time()
        self.ip = ip
        self.connection_lost = False

    @property
    def attitude(self) -> np.ndarray:
        """
        Функция вернет ndarray (6,) с координатами x, y, z, vx, vy, vz
        :return: np.ndarray
        """
        return self._attitude

    @attitude.setter
    def attitude(self, attitude) -> None:
        """
        Сеттер для attitude
        :return: None
        """
        self._attitude = attitude

    def arm(self) -> None:
        """
        Включает двигатели
        :return: None
        """
        self._send_command_long(command_name='ARM',
                                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                param1=1,
                                mavlink_send_number=self._mavlink_send_number)

    def disarm(self) -> None:
        """
        Отключает двигатели
        :return: None
        """
        self._send_command_long(command_name='DISARM',
                                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                param1=0,
                                mavlink_send_number=self._mavlink_send_number)

    def takeoff(self) -> None:
        """
        Взлет дрона
        :return: None
        """
        self._send_command_long(command_name='TAKEOFF',
                                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                mavlink_send_number=self._mavlink_send_number)

    def land(self) -> None:
        """
        Посадка дрона
        :return: None
        """
        self._send_command_long(command_name='LAND',
                                command=mavutil.mavlink.MAV_CMD_NAV_LAND,
                                mavlink_send_number=self._mavlink_send_number)

    def goto(self, x: float | int,
             y: float | int,
             z: float | int,
             yaw: float | int = 0) -> None:
        """
        Полет к указанной точке в текущей системе координат навигации.

        :param x: Координата по оси X в ENU (East-North-Up) системе координат.
        :type x: float | int
        :param y: Координата по оси Y в ENU (East-North-Up) системе координат.
        :type y: float | int
        :param z: Координата по оси Z (высота) в ENU (East-North-Up) системе координат.
        :type z: float | int
        :param yaw: Угол курса, на который должен повернуться дрон. По умолчанию 0.
        :type yaw: float | int, optional
        :return: None
        :note: Координаты задаются в ENU (East-North-Up) системе координат, но будут автоматически преобразованы 
        в NED (North-East-Down).
        """        
        mask = 0b0000_10_0_111_111_000
        x, y, z = y, x, -z
        self._send_position_target_local_ned(coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                             mask=mask, 
                                             x=x, 
                                             y=y, 
                                             z=z, 
                                             yaw=yaw, 
                                             mavlink_send_number=10)

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
                                             mask=mask,
                                             vx=vx, 
                                             vy=vy, 
                                             vz=vz, 
                                             yaw_rate=yaw_rate, 
                                             mavlink_send_number=1)

    def _send_position_target_local_ned(self, coordinate_system, 
                                        mask=0b0000_11_0_111_111_111,
                                        x: float | int = 0,
                                        y: float | int = 0,
                                        z: float | int = 0,
                                        vx: float | int = 0,
                                        vy: float | int = 0,
                                        vz: float | int = 0,
                                        afx: float | int = 0,
                                        afy: float | int = 0,
                                        afz: float | int = 0,
                                        yaw: float | int = 0,
                                        yaw_rate: float | int = 0,
                                        target_system = None,
                                        target_component = None,
                                        mavlink_send_number = 1) -> None:
        """
        Функция отправляет команду MAVLink для установки целевой позиции
        или скорости в локальной системе координат NED (North, East, Down). 
        Параметры включают систему координат, маску для указания активных полей,
        координаты (x, y, z), скорости (vx, vy, vz), ускорения и скорость поворота
        по оси yaw.
        :param coordinate_system: Система координат (например, NED).
        :type coordinate_system: int
        :param mask: Маска для включения или исключения определенных параметров.
        :type mask: int
        :param x: Координата x.
        :type x: float | int
        :param y: Координата y.
        :type y: float | int
        :param z: Координата z.
        :type z: float | int
        :param vx: Скорость по оси x.
        :type vx: float | int
        :param vy: Скорость по оси y.
        :type vy: float | int
        :param vz: Скорость по оси z.
        :type vz: float | int
        :param afx: Ускорение по оси x.
        :type afx: float | int
        :param afy: Ускорение по оси y.
        :type afy: float | int
        :param afz: Ускорение по оси z.
        :type afz: float | int
        :param yaw: Угол курса.
        :type yaw: float | int
        :param yaw_rate: Скорость изменения курса.
        :type yaw_rate: float | int
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
                           command_name,
                           command, 
                           param1: float | int = 0, 
                           param2: float | int = 0, 
                           param3: float | int = 0,
                           param4: float | int = 0,
                           param5: float | int = 0,
                           param6: float | int = 0,
                           param7: float | int = 0,
                           target_system=None,
                           target_component=None, 
                           mavlink_send_number: int = 1) -> None:
        """
        Отправляет команду типа COMMAND_LONG через MAVLink.
        :param command_name: Имя команды для логирования.
        :type command_name: str
        :param command: Команда MAVLink.
        :type command: int
        :param param1: Параметр 1 команды.
        :type param1: float | int
        :param param2: Параметр 2 команды.
        :type param2: float | int
        :param param3: Параметр 3 команды.
        :type param3: float | int
        :param param4: Параметр 4 команды.
        :type param4: float | int
        :param param5: Параметр 5 команды.
        :type param5: float | int
        :param param6: Параметр 6 команды.
        :type param6: float | int
        :param param7: Параметр 7 команды.
        :type param7: float | int
        :param target_system: Идентификатор целевой системы.
        :type target_system: int, optional
        :param target_component: Идентификатор целевого компонента.
        :type target_component: int, optional
        :param mavlink_send_number: Количество отправок команды.
        :type mavlink_send_number: int
        :return: None
        """
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
        Отправляет сообщение HEARTBEAT для поддержания активного соединения с дроном.
        :return: None
        """
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               0, 
                                               0,
                                               0)
        self._heartbeat_send_time = time.time()

    def _message_handler(self, combine_system: int=0) -> None:

        """   
        Обрабатывает сообщения от дрона и отправляет heartbeat, обновляя координаты дрона.
        :param combine_system: Определяет, с каких источников будут считываться данные:
                            0 — только локус, 1 — локус и оптика, 2 — только оптика.
        :type combine_system: int
        :return: None
        """
        if combine_system == 0:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                self.heartbeat()
                self._msg = self.mavlink_socket.recv_msg()
                if self._msg is not None:
                    if self._msg.get_type() == "LOCAL_POSITION_NED" and self._msg._header.srcComponent == 1:
                        self.attitude = np.array([self._msg.x, self._msg.y, self._msg.z, self._msg.vx, self._msg.vy, self._msg.vz])
                    elif self._msg.get_type() == "BATTERY_STATUS":
                        self.battery_voltage = self._msg.voltages[0] / 100

        elif combine_system == 1:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                self.heartbeat() 
                self._msg = self.mavlink_socket.recv_msg()
                if self._msg is not None:
                    if self._msg.get_type() == "LOCAL_POSITION_NED":
                        self.attitude = np.array([self._msg.x, self._msg.y, self._msg.z, self._msg.vx, self._msg.vy, self._msg.vz])
                    elif self._msg.get_type() == "BATTERY_STATUS":
                        self.battery_voltage = self._msg.voltages[0] / 100


        elif combine_system == 2:
            while True:
                if not self.__is_socket_open.is_set():
                    break
                self.heartbeat() 
                self._msg = self.mavlink_socket.recv_msg()
                if self._msg is not None:
                    if self._msg.get_type() == "LOCAL_POSITION_NED" and self._msg._header.srcComponent == 26:
                        self.attitude = np.array([self._msg.x, self._msg.y, self._msg.z, self._msg.vx, self._msg.vy, self._msg.vz])
                    elif self._msg.get_type() == "BATTERY_STATUS":
                        self.battery_voltage = self._msg.voltages[0] / 100

    def heartbeat(self) -> None:
        """
        Функция проверки heartbeat дрона
        :return: None
        """
        if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
            self._send_heartbeat()

    def v_while(self, ampl: float | int = 1) -> None:
        """
        Функция задает цикл while на отправку вектора скорости в body с периодом period_send_v
        :param ampl: Амплитуда усиления вектора скорости
        :type ampl: float | int
        :return: None
        """
        while self.speed_flag:
            t_speed = self.t_speed * ampl
            self.send_speed(t_speed[0], t_speed[1], t_speed[2], t_speed[3])
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
                                param1=1,
                                mavlink_send_number=self._mavlink_send_number)

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

    def led_control(self, led_id=255, r=0, g=0, b=0) -> None:
        """
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
        if led_id not in [255, 0, 1, 2, 3]:
            raise ValueError(
                f"Argument 'led_id' must have one of the following values: 0, 1, 2, 3, 255. But your value is {led_id}.")
        if r < 0 or r > 255 or g < 0 or g > 255 or b < 0 or b > 255:
            raise ValueError(
                f"Arguments 'r', 'g', 'b' must have value in [0, 255]. But your values is r={r}, g={g}, b={b}.")
        self._send_command_long(command_name='LED', command=mavutil.mavlink.MAV_CMD_USER_1,
                                       param1=led_id, param2=r, param3=g, param4=b)
    
    def check_battery(self) -> None:
        """
        Проверяет статус батареи
        :return: None
        """
        voltage = self.battery_voltage
        if voltage is not None:
            if voltage < 6.9:
                print(f">>>>>>>>>>>>>>>>>>Аккумулятор разряжен<<<<<<<<<<<<<<<<<<<<<<\nvoltage = {voltage}")
            else:
                print(f"voltage = {voltage}")
        else:
            print("Сообщение о статусе батареи еще не пришло")
