import socket
import time
from .datagram import DDatagram  
from queue import Queue
import threading
from typing import Any, Dict, Tuple

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
    def __init__(self, port: int = 37020) -> None:
        self.encoder: DDatagram = DDatagram()
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
    def __init__(self, server_to_agent_queue: Queue[Any], port: int = 37020) -> None:
        self.encoder: DDatagram = DDatagram()
        self.decoder: DDatagram = DDatagram()
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
        self.broadcast_client = UDPBroadcastClient(port=self.broadcast_port)
        self.broadcast_server = UDPBroadcastServer(server_to_agent_queue=self.receive_queue, port=self.broadcast_port)
        self.running: bool = True

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

    def process_incoming_state(self, state: Any) -> None:
        """
        Обрабатывает входящие данные о состоянии другого дрона.
        
        :param state: Данные о состоянии (например, protobuf-сообщение или его словарное представление).
        """
        print("Received state from swarm node:", state)

    def stop(self) -> None:
        """
        Останавливает работу коммуникатора.
        """
        self.running = False
        self.broadcast_server.running = False
    
    def process_incoming_state(self, state: any) -> None:
        """
        Обрабатывает входящие сообщения.
        Если поле command ненулевое – интерпретирует сообщение как команду управления.
        Если присутствует поле target_ip, команда выполняется только если target_ip совпадает с IP устройства.
        """
        if hasattr(state, "command") and state.command != 0:
            # Если в сообщении задано поле target_ip (не пустое), фильтруем команду по IP
            if hasattr(state, "target_ip") and state.target_ip:
                local_ip = self.control_object.ip
                if state.target_ip != local_ip:
                    print(f"Команда с target_ip {state.target_ip} не предназначена для этого устройства ({local_ip}). Игнорируется.")
                    return

            # Обработка команд
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
            pass
          #  print("Получено обновление состояния:", state)
