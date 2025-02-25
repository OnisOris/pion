import socket
import time
from .datagram import DDatagram  
from queue import Queue
import threading
from typing import Any, Dict, Tuple, Union
import random
import numpy as np
from .functions import vector_reached

# Определим те же коды команд
CMD_SET_SPEED = 1
CMD_GOTO      = 2
CMD_TAKEOFF   = 3
CMD_LAND      = 4
CMD_ARM       = 5
CMD_DISARM    = 6
CMD_SMART_GOTO = 7


class UDPBroadcastClient:
    """
    Клиент для отправки UDP широковещательных сообщений.
    """
    def __init__(self,
                 port: int = 37020,
                 id: int = random.randint(0, int(1e12))) -> None:
        self.encoder: DDatagram = DDatagram(id=id)
        self.port: int = port
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
        Сериализует и отправляет данные по широковещательной рассылке.
        
        :param state: Словарь с данными состояния дрона.
                      Ожидаемые ключи:
                        - 'id': идентификатор дрона (строка, будет преобразована в целое число)
                        - 'ip': IP-адрес (строка, можно не отправлять, если не требуется)
                        - 'position': список координат (например, [x, y, z])
                        - 'attitude': список углов (например, [roll, pitch, yaw])
        """
        try:
            # Преобразуем строковый id в целое число (например, через hash)
            numeric_id = abs(hash(state.get("id", "0"))) % (10**12)
            self.encoder.token = state.get("token", -1)  # можно оставить дефолтное значение
            self.encoder.id = numeric_id
            # Допустим, поле source можно заполнить, например, через преобразование IP (или оставить 0)
            self.encoder.source = 0  
            # Команда (command) оставляем 0, если нет специальной команды
            self.encoder.command = 0  
            # Объединяем данные position и attitude в одно поле data
            pos = state.get("position", [])
            att = state.get("attitude", [])
            # Если ip требуется передать как число, можно использовать, например, ipaddress.IPv4Address
            ip_str = state.get("ip", "0.0.0.0")
            try:
                import ipaddress
                ip_num = int(ipaddress.IPv4Address(ip_str))
            except Exception:
                ip_num = 0
            # Записываем в data: [ip_num] + pos + att
            self.encoder.data = [ip_num] + pos + att

            serialized: bytes = self.encoder.export_serialized()
            self.socket.sendto(serialized, ("<broadcast>", self.port))
        except Exception as error:
            print("Error sending broadcast message:", error)

class UDPBroadcastServer:
    """
    Сервер для приема UDP широковещательных сообщений.
    """
    def __init__(self, server_to_agent_queue: Queue[Any], port: int = 37020, id: int = random.randint(0, int(1e12))) -> None:
        self.encoder: DDatagram = DDatagram(id=id)
        self.decoder: DDatagram = DDatagram(id=id)
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
        Запускает сервер для постоянного приема сообщений.
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

class SwarmCommunicator:
    """
    Компонент для организации обмена данными в роевой архитектуре.
    На каждом дроне (например, объект Pion) запускается SwarmCommunicator,
    который периодически рассылает своё состояние (position, attitude, ip, id)
    и принимает аналогичные данные от других дронов.
    """
    def __init__(self,
                 control_object: Any,
                 broadcast_port: int = 37020, 
                 broadcast_interval: float = 0.05,
                 safety_radius: float = 1.,
                 max_speed: float = 1.) -> None:
        """
        :param control_object: Экземпляр дрона (например, объект Pion), из которого берутся данные состояния.
        :param control_object: Порт для широковещательной рассылки.
        :param broadcast_interval: Интервал между отправками состояния.
        """
        self.control_object = control_object
        self.broadcast_interval = broadcast_interval
        self.broadcast_port = broadcast_port
        self.receive_queue: Queue[Any] = Queue()
        self.broadcast_client = UDPBroadcastClient(port=self.broadcast_port, id=int(self.control_object.ip[-3:]))
        self.broadcast_server = UDPBroadcastServer(server_to_agent_queue=self.receive_queue, port=self.broadcast_port, id=int(self.control_object.ip[-3:]))
        self.running: bool = True
        self.env = {}
        self.safety_radius = safety_radius
        self.max_speed = max_speed

    def start(self) -> None:
        """
        Запускает два параллельных потока: один для отправки своего состояния,
        второй – для приема сообщений от других дронов.
        """
        self.broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.broadcast_thread.start()
        self.receive_thread.start()

    def _broadcast_loop(self) -> None:
        """
        Цикл отправки собственного состояния дрона.

        :return: None
        """
        while self.running:
            try:
                state: Dict[str, Any] = {
                    "id": self.control_object.name,        # преобразуется в числовой id внутри send()
                    "ip": self.control_object.ip,          # ip-адрес в виде строки
                    "position": self.control_object.position.tolist(),  # преобразуем numpy array в список
                    "attitude": self.control_object.attitude.tolist()
                }
                self.broadcast_client.send(state)
            except Exception as error:
                print("Error in broadcast loop:", error)
            time.sleep(self.broadcast_interval)

    def _receive_loop(self) -> None:
        """
        Цикл обработки входящих сообщений о состоянии других дронов.
        Запускает сервер для приема сообщений и периодически проверяет очередь.
        """
        server_thread = threading.Thread(target=self.broadcast_server.start, daemon=True)
        server_thread.start()
        while self.running:
            # try:
            if not self.receive_queue.empty():
                incoming_state = self.receive_queue.get(block=False)
                self.process_incoming_state(incoming_state)
            # except Exception as error:
            #     print("Error in receive loop:", error)
            time.sleep(0.05)

    def stop(self) -> None:
        """
        Останавливает работу коммуникатора.
        :return: None
        """
        self.running = False
        self.broadcast_server.running = False
    
 
    def process_incoming_state(self, state: Any) -> None:
        """
        Обрабатывает входящие сообщения.
        Если поле command ненулевое – интерпретирует сообщение как команду управления.
        Если присутствует поле target_ip, команда выполняется только если target_ip совпадает с IP устройства.
        Если же target_ip не соответствует, данные сохраняются в словарь self.env для будущей обработки.
        :param state: 
        :return: None
        """
        # print("V = ", self.compute_swarm_velocity())
        if not hasattr(self, "env"):
            self.env = {}  # инициализируем, если ещё не создано
        # print(self.env)
        if hasattr(state, "command") and state.command != 0:
            # Если в сообщении задано поле target_ip (не пустое), фильтруем команду по IP
            if hasattr(state, "target_ip") and state.target_ip:
                local_ip = self.control_object.ip
                if state.target_ip != local_ip:
                    print(f"Команда с target_ip {state.target_ip} не предназначена для этого устройства ({local_ip}). Данные сохранены.")
                    # Сохраняем данные в словарь self.env, ключом можно использовать, например, state.id
                    self.env[state.id] = state
                    return

            # Обработка команд, если target_ip соответствует
            if state.command == CMD_SET_SPEED:
                try:
                    vx, vy, vz, yaw_rate = state.data
                    self.control_object.send_speed(vx, vy, vz, yaw_rate)
                    print(f"Команда set_speed выполнена: {vx}, {vy}, {vz}, {yaw_rate}")
                except Exception as e:
                    print("Ошибка при выполнении set_speed:", e)
            elif state.command == CMD_GOTO:
                try:
                    x, y, z, yaw = state.data
                    self.control_object.goto(x, y, z, yaw)
                    print(f"Команда goto выполнена: {x}, {y}, {z}, {yaw}")
                except Exception as e:
                    print("Ошибка при выполнении goto:", e)
            elif state.command == CMD_TAKEOFF:
                self.control_object.takeoff()
                print("Команда takeoff выполнена")
            elif state.command == CMD_LAND:
                self.control_object.land()
                print("Команда land выполнена")
            elif state.command == CMD_ARM:
                self.control_object.arm()
                print("Команда arm выполнена")
            elif state.command == CMD_DISARM:
                self.control_object.disarm()
                print("Команда disarm выполнена")
            elif state.command == CMD_SMART_GOTO:
                self.point_reached = True
                x, y, z, yaw = state.data
                self.smart_goto(x, y, z, yaw)
                print(f"SmartGoto: ({x}, {y}, {z}), yaw={yaw}")
                # except Exception as e:
                #     print(f"Ошибка SmartGoto: {str(e)}")
            else:
                print("Получена неизвестная команда:", state.command)
        else:
            # Если поле command равно 0 – можно сохранить обновление состояния в self.env для роевого алгоритма
            # Ключ можно сформировать, например, по state.id или другому уникальному идентификатору
            if hasattr(state, "id"):
                self.env[state.id] = state
            # Или, если нет command, можно добавить по IP, если оно есть
            elif hasattr(state, "ip"):
                self.env[state.ip] = state
            # Иначе просто игнорировать
            #  print("Получено обновление состояния:", state)
    def compute_swarm_velocity(self, target_point = np.array([0, 0])) -> np.ndarray:
        """
        Вычисляет желаемый вектор скорости для локального дрона на основе информации из self.env.
        Логика:
          - Attraction: направлен от текущей позиции к средней позиции остальных дронов.
          - Repulsion: суммарное отталкивание от дронов, находящихся ближе, чем safety_radius.
          - Новый вектор = current_velocity + attraction + 4 * repulsion,
            затем ограничивается по ускорению и по максимальной скорости.

        :return: numpy-массив [vx, vy]
        :rtype: np.ndarray
        """
        local_pos = self.control_object.position[0:2]
        current_velocity = self.control_object.position[3:5]  # np.array([vx, vy])
        # Attraction force: единичный вектор от текущей позиции к swarm_goal
        direction = target_point - local_pos
        norm_dir = np.linalg.norm(direction)
        if norm_dir != 0:
            attraction_force = direction / norm_dir
        else:
            attraction_force = np.zeros(2)
        # Repulsion force: суммируем вклад от каждого дрона, если расстояние меньше safety_radius
        repulsion_force = np.zeros(2)
        for state in self.env.values():
            if len(state.data) >= 3:
                other_pos = np.array(state.data[1:3], dtype=float)
                distance_vector = local_pos - other_pos
                distance = np.linalg.norm(distance_vector)
                if 0 < distance < self.safety_radius:
                    repulsion_force += distance_vector / (distance ** 2)
        # Вычисляем новый вектор скорости (базовый алгоритм)
        new_velocity = current_velocity + attraction_force + 4 * repulsion_force + np.array([np.random.rand() * 0.4,
                                                                                             np.random.rand() * 0.4])
        # Ограничиваем изменение (акселерацию) до max_acceleration (например, 0.15)
        new_velocity = self._limit_acceleration(current_velocity, new_velocity, max_acceleration=0.1)
        # Ограничиваем скорость до self.max_speed
        new_velocity = self._limit_speed(new_velocity)
        return new_velocity

    def _limit_acceleration(self, 
                            current_velocity: np.ndarray,
                            target_velocity: np.ndarray,
                            max_acceleration: float) -> np.ndarray:
        """
        Метод ограничения максимального ускорения
        :param current_velocity: массив скоростей
        :type current_velocity: np.ndarray

        :param target_velocity: целевая скорость
        :type target_velocity: np.ndarray
        :param max_acceleration: предел по максимальному ускорению
        :type max_acceleration: float

        :return: максимальное изменение скорости
        :rtype: np.ndarray
        """
        change = target_velocity - current_velocity
        norm = np.linalg.norm(change)
        if norm > max_acceleration:
            change = change / norm * max_acceleration
        return current_velocity + change

    def _limit_speed(self, 
                     velocity: np.ndarray) -> np.ndarray:
        """
        Метод для ограничения скорости.
        В диапазоне (0.03, 0.45) скорость делится на 1.4.
        В остальном ограничение идет по self.max_speed.
        
        :param velocity: Ограничеваемая скорость
        :type velocity: np.ndarray

        :return: np.ndarray
        :rtype: np.ndarray
        """
        norm = np.linalg.norm(velocity)
        if 0.03 < norm < 0.45:
            return velocity / 1.4
        elif norm < 0.06:
            return np.zeros_like(velocity)
        if norm > self.control_object.max_speed:
            return velocity / norm * self.control_object.max_speed
        return velocity

    def update_swarm_control(self, target_point) -> None:
        """
        Вычисляет новый вектор скорости для локального дрона на основе информации из self.env
        и записывает его в self.control_object.t_speed.

        :return: None
        :rtype: None
        """
        new_vel = self.compute_swarm_velocity(target_point)
        self.control_object.t_speed = np.array([new_vel[0], new_vel[1], 0, 0])

    def smart_goto(self,
                    x: Union[float, int],
                    y: Union[float, int],
                    z: Union[float, int],
                    yaw: Union[float, int] = 0,
                    accuracy: Union[float, int] = 5e-2) -> None:
        """
        Функция, выполняющая перемещение к точке безопасным образом, облетая дроны
        :param x: координата по x
        :type x: Union[float, int]
        :param y: координата по y
        :type y: Union[float, int]
        :param z: координата по z (Пока не учавствует в рассчетах)
        :type z: Union[float, int]
        :param yaw: координата по yaw
        :type yaw: Union[float, int]
        :param accuracy: Погрешность целевой точки
        :type accuracy: Union[float, int, None]
        :return: None
        """
        print(f"Smart goto to {x, y, z, yaw}")
        self.control_object.set_v()
        self.control_object.goto_yaw(yaw)
        target_point = np.array([x, y])
        self.control_object.point_reached = False
        # last_time = time.time() # Закоменченное позже понадобится в алгоритмах 
        time.sleep(self.control_object.period_send_speed)
        while not self.control_object.point_reached:
            self.control_object.point_reached = vector_reached(target_point, #TODO: Сделать остановку по скорости
                                                               self.control_object.last_points[:,:2],
                                                               accuracy=accuracy) and np.allclose(self.control_object.position[3:6], np.array([0, 0, 0]), atol=1e-4)
            self.update_swarm_control(np.array([x, y]))
            time.sleep(self.control_object.period_send_speed)
        print("smart end")
        self.t_speed = np.zeros(4)
        



