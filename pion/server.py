import socket
import time
from .datagram import DDatagram  
from queue import Queue
import threading
from typing import Any, Dict, Tuple
import random
import numpy as np

# Определим те же коды команд
CMD_SET_SPEED = 1
CMD_GOTO      = 2
CMD_TAKEOFF   = 3
CMD_LAND      = 4
CMD_ARM       = 5
CMD_DISARM    = 6


class UDPBroadcastClient:
    """
    Клиент для отправки UDP широковещательных сообщений.
    """
    def __init__(self, port: int = 37020, id: int = random.randint(0, int(1e12))) -> None:
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
    def __init__(self, control_object: Any, broadcast_port: int = 37020, broadcast_interval: float = 0.5) -> None:
        """
        :param pion: Экземпляр дрона (например, объект Pion), из которого берутся данные состояния.
        :param broadcast_port: Порт для широковещательной рассылки.
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
        self.safety_radius = 1

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
        """
        while self.running:
            try:
                # Собираем данные состояния дрона.
                # Здесь предполагается, что pion имеет атрибуты name, ip, position и attitude.
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
            try:
                if not self.receive_queue.empty():
                    incoming_state = self.receive_queue.get(block=False)
                    self.process_incoming_state(incoming_state)
            except Exception as error:
                print("Error in receive loop:", error)
            time.sleep(0.1)


    def stop(self) -> None:
        """
        Останавливает работу коммуникатора.
        """
        self.running = False
        self.broadcast_server.running = False
    
 
    def process_incoming_state(self, state: Any) -> None:
        """
        Обрабатывает входящие сообщения.
        Если поле command ненулевое – интерпретирует сообщение как команду управления.
        Если присутствует поле target_ip, команда выполняется только если target_ip совпадает с IP устройства.
        Если же target_ip не соответствует, данные сохраняются в словарь self.env для будущей обработки.
        """
        print("V = ", self.compute_swarm_velocity())
        if not hasattr(self, "env"):
            self.env = {}  # инициализируем, если ещё не создано
        print(self.env)
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
    def compute_swarm_velocity(self) -> np.ndarray:
        """
        Вычисляет желаемый вектор скорости для локального дрона на основе информации из self.env.
        Логика:
          - Attraction: направлен от текущей позиции к средней позиции остальных дронов.
          - Repulsion: суммарное отталкивание от дронов, находящихся ближе, чем safety_radius.
          - Новый вектор = current_velocity + attraction + 4 * repulsion,
            затем ограничивается по ускорению и по максимальной скорости.
        Возвращает numpy-массив [vx, vy].
        """
        local_pos = self.control_object.position[0:2]
        current_velocity = self.control_object.position[3:5]  # np.array([vx, vy])

        # Если env пуст, то цель – текущая позиция (нет притяжения)
        positions = []
        for state in self.env.values():
            if len(state.data) >= 3:
                # Предполагается, что state.data[1] и state.data[2] – x и y
                positions.append(np.array(state.data[1:3], dtype=float))
        if positions:
            # Средняя позиция остальных дронов
            swarm_goal = np.mean(positions, axis=0)
        else:
            swarm_goal = local_pos

        # Attraction force: единичный вектор от текущей позиции к swarm_goal
        direction = swarm_goal - local_pos
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
        new_velocity = current_velocity + attraction_force + 4 * repulsion_force

        # Ограничиваем изменение (акселерацию) до max_acceleration (например, 0.15)
        new_velocity = self._limit_acceleration(current_velocity, new_velocity, max_acceleration=0.15)
        # Ограничиваем скорость до self.max_speed
        new_velocity = self._limit_speed(new_velocity)
        return new_velocity

    def _limit_acceleration(self, current_velocity, target_velocity, max_acceleration):
        change = target_velocity - current_velocity
        norm = np.linalg.norm(change)
        if norm > max_acceleration:
            change = change / norm * max_acceleration
        return current_velocity + change

    def _limit_speed(self, velocity):
        norm = np.linalg.norm(velocity)
        if 0.03 < norm < 0.45:
            return velocity / 1.4
        elif norm < 0.06:
            return np.zeros_like(velocity)
        if norm > self.control_object.max_speed:
            return velocity / norm * self.control_object.max_speed
        return velocity

    def update_swarm_control(self) -> None:
        """
        Вычисляет новый вектор скорости для локального дрона на основе информации из self.env
        и записывает его в self.control_object.t_speed.
        """
        new_vel = self.compute_swarm_velocity()
        self.control_object.t_speed = new_vel
        print(f"Swarm control updated t_speed: {new_vel}")

