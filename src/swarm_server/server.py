from __future__ import annotations
import datetime
import os
import random
import socket
import threading
import time
from queue import Queue
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np
from lokky.pionmath import SSolver

from pion.pio import DroneBase
from pionfunc.annotation import Array6
from pionfunc.functions import (
    get_local_ip,
    get_numeric_id,
    get_unique_instance_id,
    start_threading,
    vector_reached,
)

from .commands import CMD
from .datagram import DDatagram

#####################################
# UDP Broadcast Client & Server
#####################################


class UDPBroadcastClient:
    """
    Клиент для отправки UDP широковещательных сообщений
    """

    def __init__(self, port: int = 37020, unique_id: int = 0) -> None:
        """
        Инициализация клиента для отправки UDP широковещательных сообщений.

        :param port: Порт для отправки сообщений.
        :type port: int

        :param unique_id: Уникальный идентификатор устройства. Если равен 0, генерируется случайное число.
        :type unique_id: int

        :return: None
        :rtype: None
        """

        numeric_id = (
            get_numeric_id(unique_id)
            if unique_id
            else random.randint(0, int(1e12))
        )
        self.encoder: DDatagram = DDatagram(id=numeric_id)
        self.port: int = port
        self.unique_id: int = unique_id
        try:
            self.socket: socket.socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
            )
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            print("UDP Broadcast Client launched")
        except Exception as error:
            print("Broadcast client initialization failure:", error)

    def send(self, state: Dict[str, Any]) -> None:
        """
        Сериализует и отправляет данные через UDP широковещательный канал.

        :param state: Словарь с данными состояния, включающий:
            - id: Идентификатор устройства.
            - token: Токен для аутентификации.
            - command: Команда для выполнения.
            - target_id (опционально): Идентификатор целевого устройства.
            - position: Список координат позиции.
            - attitude: Список углов наклона.
            - t_speed: Список значений скорости.
        :type state: Dict[str, Any]

        :return: None
        """
        try:
            # Преобразуем строковый id в числовой (для DDatagram)
            numeric_id = get_numeric_id(state.get("id", "0"))
            self.encoder.token = state.get("token", -1)
            self.encoder.id = numeric_id
            self.encoder.source = 0
            self.encoder.command = state.get("command", 0)
            if "target_id" in state:
                self.encoder.target_id = state[
                    "target_id"
                ]  # Используем строковый target_id

            pos = state.get(
                "position", [0.0, 0.0, 0.0]
            )  # Значения по умолчанию
            att = state.get("attitude", [0.0, 0.0, 0.0])
            t_speed = state.get("t_speed", [0.0, 0.0, 0.0, 0.0])
            try:
                ip_num = int(self.unique_id)
            except Exception:
                ip_num = 0
            self.encoder.data = [ip_num] + pos + att + t_speed
            serialized: bytes = self.encoder.export_serialized()
            self.socket.sendto(serialized, ("<broadcast>", self.port))
        except Exception as error:
            print("Error sending broadcast message:", error)


class UDPBroadcastServer:
    """
    Сервер для приема UDP широковещательных сообщений.
    """

    def __init__(
        self,
        server_to_agent_queue: Queue[Any],
        port: int = 37020,
        unique_id: Optional[int] = None,
    ) -> None:
        """
        Инициализация сервера для приёма UDP широковещательных сообщений.

        :param server_to_agent_queue: Очередь для передачи полученных сообщений.
        :type server_to_agent_queue: Queue[Any]

        :param port: Порт для приёма сообщений.
        :type port: int

        :param unique_id: Уникальный идентификатор устройства.
        :type unique_id: Optional[int]
        """
        numeric_id = (
            get_numeric_id(unique_id)
            if unique_id
            else random.randint(0, int(1e12))
        )
        self.encoder: DDatagram = DDatagram(id=numeric_id)
        self.decoder: DDatagram = DDatagram(id=numeric_id)
        self.server_to_agent_queue: Queue[Any] = server_to_agent_queue
        self.port: int = port
        try:
            self.socket: socket.socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
            )
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.bind(("", self.port))
            print("UDP Broadcast Server launched on port", self.port)
        except Exception as error:
            print("Broadcast server start failure:", error)
        self.running: bool = True

    def start(self) -> None:
        """
        Запускает сервер для постоянного приёма UDP сообщений.

        :return: None
        :rtype: None
        """
        while self.running:
            try:
                message: bytes
                sender_address: Tuple[str, int]
                message, sender_address = self.socket.recvfrom(4096)
                is_valid, payload = self.decoder.read_serialized(message)
                if is_valid:
                    self.server_to_agent_queue.put(payload, block=False)
                else:
                    print("Received invalid datagram from", sender_address)
            except Exception as error:
                print("Datagram reception error:", error)
                time.sleep(0.1)


#####################################
# SwarmCommunicator
#####################################


class SwarmCommunicator:
    """
    Компонент для обмена данными в роевой архитектуре.

    Каждый экземпляр получает уникальный идентификатор на основе IP и порядкового номера.
    """

    def __init__(
        self,
        control_object: Any,
        broadcast_port: int = 37020,
        broadcast_interval: float = 0.5,
        recive_interval: float = 0.05,
        safety_radius: float = 1.0,
        max_speed: float = 1.0,
        ip: Optional[str] = None,
        instance_number: Optional[Any] = None,
        time_sleep_update_velocity: float = 0.1,
        params: Optional[dict] = None,
        mode: int = 1,
        unique_id: int = None,
        use_multicast: bool = True,
        multicast_group: str = "224.0.0.1",
    ) -> None:
        """
        Инициализация компонента для обмена данными в роевой архитектуре.

        :param control_object: Объект управления дроном.
        :type control_object: Any

        :param broadcast_port: Порт для UDP широковещательных сообщений.
        :type broadcast_port: int

        :param broadcast_interval: Интервал отправки сообщений (в секундах).
        :type broadcast_interval: float

        :param recive_interval: Интервал проверки входящих сообщений (в секундах).
        :type recive_interval: float

        :param safety_radius: Радиус безопасности.
        :type safety_radius: float

        :param max_speed: Максимальная скорость.
        :type max_speed: float

        :param ip: IP-адрес устройства (если None, используется значение из control_object).
        :type ip: Optional[str]

        :param instance_number: Порядковый номер экземпляра для генерации уникального идентификатора.
        :type instance_number: Optional[Any]

        :param time_sleep_update_velocity: Интервал обновления вектора скорости (в секундах).
        :type time_sleep_update_velocity: float

        :param params: Дополнительные параметры алгоритма.
        :type params: Optional[dict]

        :param mode: Режим роя, имеет 3 состояния: 1 - включен ПИД до таргетной позиции + силы отталкивания от других
         донов + вектор вывода дрона с состояния равновесия в локальных минимумах

        :param use_multicast: Ипользование мультикаста
        :type use_multicast: bool

        :param multicast_group: Адрес мультикаст группы
        :type multicast_group: str

        :return: None
        :rtype: None
        """
        self.group_id: int = 0
        if params is None:
            self.params = {
                "kp": np.array([1, 1, 1, 1, 1, 1]),
                "ki": np.zeros((1, 6)),
                "kd": np.array([1, 1, 1, 1, 1, 1]),
                "attraction_weight": 1.0,
                "cohesion_weight": 1.0,
                "alignment_weight": 1.0,
                "repulsion_weight": 4.0,
                "unstable_weight": 1.0,
                "noise_weight": 1.0,
                "safety_radius": 1.0,
                "max_acceleration": 1,
                "max_speed": 0.4,
                "unstable_radius": 1.5,
            }
        else:
            self.params = params
        self.swarm_solver = SSolver(params=self.params, count_of_objects=1)

        self.mode: int = mode
        self.control_object: DroneBase = control_object
        self.env_state_matrix: np.ndarray = np.array(
            [self.control_object.position]
        )
        self.broadcast_interval = broadcast_interval
        self.broadcast_port = broadcast_port
        self.receive_queue: Queue[Any] = Queue()
        # Определяем IP для формирования уникального id
        local_ip = ip if ip is not None else self.control_object.ip
        if unique_id:
            self.unique_id = unique_id
        else:
            self.unique_id: int = (
                get_unique_instance_id(
                    local_ip, instance_number=instance_number
                )
                if ip != "localhost"
                else control_object.mavlink_port
            )
        print(
            "Уникальный id для этого экземпляра:", self.unique_id, " ip = ", ip
        )
        self.numeric_id = get_numeric_id(self.unique_id)
        self.broadcast_client = UDPBroadcastClient(
            port=self.broadcast_port, unique_id=self.unique_id
        )
        self.broadcast_server = UDPBroadcastServer(
            server_to_agent_queue=self.receive_queue,
            port=self.broadcast_port,
            unique_id=self.unique_id,
        )
        self.running: bool = True
        self.env = {}
        self.safety_radius = safety_radius
        self.max_speed = max_speed
        self.time_sleep_update_velocity = time_sleep_update_velocity
        self.recive_interval = recive_interval
        self.control_object.name += (
            f", unid: {self.unique_id}, gr: {self.group_id}"
        )

    @property
    def co(self):
        """
        Короткий геттер контролируемого объекта
        """
        return self.control_object

    def start(self) -> None:
        """
        Запускает параллельные потоки для рассылки собственного состояния и приёма сообщений от других устройств.

        :return: None
        :rtype: None
        """
        self.broadcast_thread = threading.Thread(
            target=self._broadcast_loop, daemon=True
        )
        self.receive_thread = threading.Thread(
            target=self._receive_loop, daemon=True
        )
        self.broadcast_thread.start()
        self.receive_thread.start()

    def _broadcast_loop(self) -> None:
        """
        Запускает цикл отправки собственного состояния через UDP широковещательный канал.

        :return: None
        :rtype: None
        """
        while self.running:
            try:
                state: Dict[str, Any] = {
                    "id": self.unique_id,
                    "ip": self.control_object.ip,
                    "position": self.control_object.position.tolist(),
                    "attitude": self.control_object.attitude.tolist(),
                    "t_speed": self.control_object.t_speed.tolist(),
                }
                self.broadcast_client.send(state)
            except Exception as error:
                print("Error in broadcast loop:", error)
            time.sleep(self.broadcast_interval)

    def _receive_loop(self) -> None:
        """
        Циклобработки входящих сообщений.

        :return: None
        :rtype: None
        """
        server_thread = threading.Thread(
            target=self.broadcast_server.start, daemon=True
        )
        server_thread.start()
        while self.running:
            if not self.receive_queue.empty():
                incoming_state = self.receive_queue.get(block=False)
                self.process_incoming_state(incoming_state)
            time.sleep(self.recive_interval)

    def stop(self) -> None:
        """
        Останавливает коммуникатор.

        :return: None
        :rtype: None
        """
        self.control_object.stop()
        self.running = False
        self.broadcast_server.running = False

    def save_data(self) -> None:
        """
        Функция для сохранения данных задержек в работе кода

        :return: None
        :rtype: None
        """
        current_date = datetime.date.today().isoformat()
        current_time = str(datetime.datetime.now().time())
        symbols_to_remove = ":"
        for symbol in symbols_to_remove:
            current_time = current_time.replace(symbol, "-")
        main_path = f"./data/{current_date}_{current_time[0:5]}/"
        os.makedirs(main_path, exist_ok=True)
        try:
            self.control_object.save_data("trajectory.npy", f"{main_path}/")
        except Exception as e:
            print(f"Ошибка сохранения данных: {e}")

    def process_incoming_state(self, state: Any) -> None:
        """
        Функция обработчик входящих сообщений

        :return: None
        :rtype: None
        """

        if state.target_id:
            if self.unique_id != int(state.target_id):
                return
        elif state.group_id:
            if state.group_id != self.group_id:
                return
        if hasattr(state, "command") and state.command != 0:
            command = CMD(state.command)

            if command == CMD.SET_SPEED:
                try:
                    vx, vy, vz, yaw_rate = state.data
                    self.control_object.send_speed(vx, vy, vz, yaw_rate)
                    print(
                        f"Команда set_speed выполнена: {vx}, {vy}, {vz}, {yaw_rate}"
                    )
                except Exception as e:
                    print("Ошибка при выполнении set_speed:", e)
            elif command == CMD.SET_GROUP:
                try:
                    new_group = int(state.data[0])
                    self.group_id = new_group
                    self.control_object.name = f"{get_local_ip()}, unid: {self.unique_id}, gr: {self.group_id}"
                    print(
                        f"Группа успешно изменена на: {new_group} для дрона с id {self.unique_id}"
                    )
                except Exception as e:
                    print("Ошибка при изменении группы:", e)
            elif command == CMD.GOTO:
                try:
                    x, y, z, yaw = state.data
                    if self.control_object.tracking:
                        print(f"Smart tracking to {x, y, z, yaw}")
                        self.control_object.target_point = np.array(
                            [x, y, z, yaw]
                        )
                    else:
                        self.stop_trp()
                        self.control_object.goto_from_outside(x, y, z, yaw)
                        print(f"Команда goto выполнена: {x}, {y}, {z}, {yaw}")
                except Exception as e:
                    print("Ошибка при выполнении goto:", e)
            elif command == CMD.TAKEOFF:
                self.stop_trp()
                self.control_object.takeoff()
                print("Команда takeoff выполнена")
            elif command == CMD.LAND:
                self.stop_trp()
                self.control_object.land()
                print("Команда land выполнена")
            elif command == CMD.ARM:
                self.stop_trp()
                self.control_object.arm()
                print("Команда arm выполнена")
            elif command == CMD.DISARM:
                self.stop_trp()
                self.control_object.disarm()
                print("Команда disarm выполнена")
            elif command == CMD.STOP:
                self.stop_trp()
                print("Команды на достижение позиций остановлены")
            elif command == CMD.SWARM_ON:
                try:
                    if self.control_object.tracking:
                        print("Режим слежения за точкой уже включен")
                    else:
                        self.control_object.tracking = True
                        start_threading(self.smart_point_tracking)
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)

            elif command == CMD.SMART_GOTO:
                try:
                    self.stop_trp()
                    x, y, z, yaw = state.data
                    self.control_object.threads.append(
                        start_threading(self.smart_goto, x, y, z, yaw)
                    )
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)
            elif command == CMD.LED:
                try:
                    led_id, r, g, b = state.data
                    self.control_object.led_control(led_id, r, g, b)
                    print(
                        f"LED control executed: led_id={led_id}, r={r}, g={g}, b={b}"
                    )
                except Exception as e:
                    print("Ошибка при выполнении LED control:", e)
            elif command == CMD.SAVE:
                self.save_data()
            elif command == CMD.SET_MOD:
                # +-------------+-----+-----------+-----------------+
                # |             | PID | REPULSION | UNSTABLE_VECTOR |
                # +=============+=====+===========+=================+
                # | SWARM_MODE  | 1   | 1         | 1               |
                # +-------------+-----+-----------+-----------------+
                # | MASTER_MODE | 1   | 0         | 0               |
                # +-------------+-----+-----------+-----------------+
                # | FLOW_MODE   | 0   | 1         | 0               |
                # +-------------+-----+-----------+-----------------+
                try:
                    self.mode = int(state.data[0])
                    if self.mode == 1:
                        self.restore_params()
                    elif self.mode == 2:
                        self.restore_params()
                        self.swarm_solver.repulsion_weight = 0
                    elif self.mode == 3:
                        self.restore_params()
                        self.swarm_solver.kp = self.params["kp"]
                        self.swarm_solver.ki = self.params["ki"]
                        self.swarm_solver.kd = self.params["kd"]
                        self.swarm_solver.unstable_weight = 0.0
                except Exception as e:
                    print("Ошибка при выполнении set_mode:", e)
            else:
                print("Получена неизвестная команда:", state.command)
        else:
            if hasattr(state, "id"):
                if not state.id == self.numeric_id:
                    self.env[state.id] = state
            elif hasattr(state, "ip"):
                self.env[state.ip] = state
            state = self.env.values()
            positions = [drone_data.data[1:7] for drone_data in state]
            self.env_state_matrix = np.vstack(
                [self.control_object.position, *positions]
            )

    def restore_params(self) -> None:
        """
        Вовзрат параметров swarm_solver в начальное значение
        """

    def update_swarm_control(self, target_position: Array6, dt: float) -> None:
        """
        Вычисление нового вектора скорости и запись его в t_speed

        :param target_position: Целевая позиция
        :param dt: Шаг времени

        :return: None
        :rtype: None
        """
        new_vel = self.swarm_solver.solve_for_one(
            state_matrix=self.env_state_matrix,
            target_position=target_position,
            dt=dt,
        )[0]

        if self.control_object.position[2] < 0.5:
            if new_vel[2] < 0:
                new_vel[2] = 0

        self.control_object.t_speed = np.array(
            [new_vel[0], new_vel[1], new_vel[2], 0]
        )

    def start_threading_smart_goto(
        self,
        x: Union[float, int],
        y: Union[float, int],
        z: Union[float, int],
        yaw: Union[float, int] = 0,
        accuracy: Union[float, int] = 5e-2,
    ) -> None:
        """
        Запуск потока слежения за точкой с алгоритмом избегания столкновений

        :return: None
        :rtype: None
        """
        thread = threading.Thread(
            target=self.smart_goto,
            args=(
                x,
                y,
                z,
                yaw,
                accuracy,
            ),
        )
        thread.start()

    def smart_goto(
        self,
        x: Union[float, int],
        y: Union[float, int],
        z: Union[float, int],
        yaw: Union[float, int] = 0,
        accuracy: Union[float, int] = 5e-2,
    ) -> None:
        """
        Метод отправляет дрон в точку [x, y, z, yaw]. с алгоритмом избегания столкновений

        :param x: Коодината x
        :param y: Коодината y
        :param z: Коодината z
        :param yaw: Коодината yaw
        :param accuracy: Точность
        :return: None
        :rtype: None
        """

        print(f"Smart goto to {x, y, z, yaw}")
        self.control_object.set_v()
        self.control_object.goto_yaw(yaw)
        target_point = np.array([x, y])
        self.control_object.point_reached = False
        last_time = time.time()
        time.sleep(self.control_object.period_send_speed)
        while not self.control_object.point_reached:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.control_object.point_reached = vector_reached(
                target_point,
                self.control_object.last_points[:, :2],
                accuracy=accuracy,
            ) and np.allclose(
                self.control_object.position[3:6],
                np.array([0, 0, 0]),
                atol=1e-2,
            )
            self.update_swarm_control(np.array([x, y, z, 0, 0, 0]), dt=dt)
            time.sleep(self.time_sleep_update_velocity)
        print("smart end")
        time.sleep(0.5)
        self.control_object.t_speed = np.zeros(4)

    def smart_point_tracking(self) -> None:
        """
        Метод включает режим слежения за точкой.

        Дрон начинает следить за точкой из target_point[0:2]

        :return: None
        :rtype: None
        """
        print("Smart point tracking")
        self.control_object.set_v()
        self.control_object.point_reached = False
        self.control_object.tracking = True
        last_time = time.time()
        time.sleep(self.control_object.period_send_speed)
        while self.control_object.tracking:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.update_swarm_control(
                np.hstack([self.control_object.target_point[0:3], [0, 0, 0]]),
                dt=dt,
            )
            time.sleep(self.time_sleep_update_velocity)
        self.t_speed = np.zeros(4)

    def stop_trp(self) -> None:
        """
        Метод останавливает потоки, отправляющие вектора скорости на дрон

        :return: None
        :rtype: None
        """
        self.control_object.stop_moving()

