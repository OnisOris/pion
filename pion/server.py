import socket
import time
from queue import Queue
import threading
import random
import numpy as np
from typing import Any, Dict, Tuple, Union, Optional
from .datagram import DDatagram  
from .functions import vector_reached, compute_swarm_velocity, compute_swarm_velocity_boids
from .commands import *



def get_unique_instance_id(ip: str, instance_number=None) -> str:
    octet = ip.split('.')[-1] if ip.count('.') == 3 else str(hash(ip) % 1000)
    return f"{octet}-{instance_number}" if instance_number else octet

def get_numeric_id(unique_id: str) -> int:
    return abs(hash(unique_id)) % (10**12)


#####################################
# UDP Broadcast Client & Server
#####################################

class UDPBroadcastClient:
    """
    Клиент для отправки UDP широковещательных сообщений.
    """
    def __init__(self, port: int = 37020, unique_id = None) -> None:
        # Если unique_id задан, преобразуем его в число для конструктора DDatagram
        numeric_id = get_numeric_id(unique_id) if unique_id else random.randint(0, int(1e12))
        self.encoder: DDatagram = DDatagram(id=numeric_id)
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
        Добавляем поле target_id, если команда адресована конкретному устройству.
        """
        try:
            # Преобразуем строковый id в числовой (для DDatagram)
            numeric_id = get_numeric_id(state.get("id", "0"))
            # print("numeric_id = ", numeric_id)
            self.encoder.token = state.get("token", -1)
            self.encoder.id = numeric_id
            self.encoder.source = 0  
            self.encoder.command = state.get("command", 0)
            if "target_id" in state:
                self.encoder.target_id = state["target_id"]  # Используем строковый target_id
                # pos = state.get("position", [])
            # att = state.get("attitude", [])
            pos = state.get("position", [0.0, 0.0, 0.0])  # Значения по умолчанию
            att = state.get("attitude", [0.0, 0.0, 0.0])
            ip_str = state.get("ip", "0.0.0.0")
            try:
                import ipaddress
                ip_num = int(ipaddress.IPv4Address(ip_str))
            except Exception:
                ip_num = 0
            self.encoder.data = [ip_num] + pos + att
            serialized: bytes = self.encoder.export_serialized()
            self.socket.sendto(serialized, ("<broadcast>", self.port))
        except Exception as error:
            print("Error sending broadcast message:", error)

class UDPBroadcastServer:
    """
    Сервер для приёма UDP широковещательных сообщений.
    """
    def __init__(self, server_to_agent_queue: Queue[Any], port: int = 37020, unique_id: Optional[str] = None) -> None:
        numeric_id = get_numeric_id(unique_id) if unique_id else random.randint(0, int(1e12))
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
        Запускает сервер для постоянного приёма сообщений.
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
    def __init__(self,
                 control_object: Any,
                 broadcast_port: int = 37020, 
                 broadcast_interval: float = 0.05,
                 safety_radius: float = 1.,
                 max_speed: float = 1.,
                 ip = None,
                 instance_number = None) -> None:
        self.control_object = control_object
        self.broadcast_interval = broadcast_interval
        self.broadcast_port = broadcast_port
        self.receive_queue: Queue[Any] = Queue()
        # Определяем IP для формирования уникального id
        local_ip = ip if ip is not None else self.control_object.ip
        # Генерируем уникальный id с использованием instance_number, если он передан
        self.unique_id: str = get_unique_instance_id(local_ip, instance_number=instance_number)
        print("Уникальный id для этого экземпляра:", self.unique_id)
        self.numeric_id = get_numeric_id(self.unique_id)
        self.broadcast_client = UDPBroadcastClient(port=self.broadcast_port, unique_id=self.unique_id)
        self.broadcast_server = UDPBroadcastServer(server_to_agent_queue=self.receive_queue, port=self.broadcast_port, unique_id=self.unique_id)
        self.running: bool = True
        self.env = {}
        self.safety_radius = safety_radius
        self.max_speed = max_speed

    def start(self) -> None:
        """
        Запускает параллельные потоки: один для рассылки собственного состояния,
        второй – для приёма сообщений от других дронов.
        """
        self.broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.broadcast_thread.start()
        self.receive_thread.start()

    def _broadcast_loop(self) -> None:
        """
        Цикл отправки собственного состояния.
        """
        while self.running:
            try:
                state: Dict[str, Any] = {
                    "id": self.unique_id,        # теперь id – уникальный строковый идентификатор
                    "ip": self.control_object.ip,
                    "position": self.control_object.position.tolist(),
                    "attitude": self.control_object.attitude.tolist()
                    # Можно добавить поле command, если требуется
                }
                self.broadcast_client.send(state)
            except Exception as error:
                print("Error in broadcast loop:", error)
            time.sleep(self.broadcast_interval)

    def _receive_loop(self) -> None:
        """
        Цикл обработки входящих сообщений.
        """
        server_thread = threading.Thread(target=self.broadcast_server.start, daemon=True)
        server_thread.start()
        while self.running:
            if not self.receive_queue.empty():
                incoming_state = self.receive_queue.get(block=False)
                self.process_incoming_state(incoming_state)
            time.sleep(0.05)

    def stop(self) -> None:
        """
        Останавливает коммуникатор.
        """
        self.running = False
        self.broadcast_server.running = False


    def process_incoming_state(self, state: Any) -> None:
        # Проверяем наличие target_id в сообщении
        if state.target_id:
            if state.target_id != self.unique_id:
                return
        if hasattr(state, "command") and state.command != 0:
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
                    if self.control_object.tracking:
                        print(f"Smart tracking to {x, y, 1.5}")
                        self.control_object.target_point= np.array([x, y, 1.5, 0])
                    else:
                        self.control_object.goto_from_outside(x, y, z, yaw)
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
            elif state.command == CMD_STOP:
                self.control_object.tracking = False
                self.control_object.point_reached = True
                self.control_object.speed_flag = False
                print("Команды на достижение позиций остановлены")
            elif state.command == CMD_SWARM_ON:
                try:
                    self.control_object.tracking = True
                    self.start_threading(self.smart_point_tacking)
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)

            elif state.command == CMD_SMART_GOTO:
                try:
                    x, y, z, yaw = state.data
                    self.start_threading(self.smart_goto, x, y, z, yaw)
                except Exception as e:
                    print("Ошибка при выполнении smart_goto:", e)
            elif state.command == CMD_LED:
                try:
                    led_id, r, g, b = state.data
                    self.control_object.led_control(led_id, r, g, b)
                    print(f"LED control executed: led_id={led_id}, r={r}, g={g}, b={b}")
                except Exception as e:
                    print("Ошибка при выполнении LED control:", e)
            else:
                print("Получена неизвестная команда:", state.command)
        else:
            if hasattr(state, "id"):
                if not state.id == self.numeric_id:
                    self.env[state.id] = state
            elif hasattr(state, "ip"):
                self.env[state.ip] = state

    def update_swarm_control(self, target_point) -> None:
        new_vel = compute_swarm_velocity(self.control_object.position, self.env, target_point)
        self.control_object.t_speed = np.array([new_vel[0], new_vel[1], 0, 0])
    
    def start_threading_smart_goto(self,
                                   x: Union[float, int],
                                   y: Union[float, int],
                                   z: Union[float, int],
                                   yaw: Union[float, int] = 0,
                                   accuracy: Union[float, int] = 5e-2):
        thread = threading.Thread(target=self.smart_goto, args=(x, y, z, yaw, accuracy,))
        thread.start()

    def start_threading(self, function, *args, **kwargs):
        thread = threading.Thread(target=function, args=args)
        thread.start()

    def smart_goto(self,
                   x: Union[float, int],
                   y: Union[float, int],
                   z: Union[float, int],
                   yaw: Union[float, int] = 0,
                   accuracy: Union[float, int] = 5e-2) -> None:
        print(f"Smart goto to {x, y, z, yaw}")
        self.control_object.set_v()
        self.control_object.goto_yaw(yaw)
        target_point = np.array([x, y])
        self.control_object.point_reached = False
        time.sleep(self.control_object.period_send_speed)
        while not self.control_object.point_reached:
            self.control_object.point_reached = vector_reached(target_point,
                                                               self.control_object.last_points[:,:2],
                                                               accuracy=accuracy) and np.allclose(
                                                                   self.control_object.position[3:6],
                                                                   np.array([0, 0, 0]),
                                                                   atol=1e-2)
            self.update_swarm_control(np.array([x, y]))
            time.sleep(self.control_object.period_send_speed)
        print("smart end")
        time.sleep(0.5)
        self.control_object.t_speed = np.zeros(4)


    def smart_point_tacking(self):
        print(f"Smart point tracking")
        self.control_object.set_v()
        self.control_object.point_reached = False
        self.control_object.tracking = True
        while self.control_object.tracking:
            self.update_swarm_control(self.control_object.target_point[0:2])
            time.sleep(self.control_object.period_send_speed)
        self.t_speed = np.zeros(4)
