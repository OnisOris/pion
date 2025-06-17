import socket
from typing import Optional

from pionfunc.functions import extract_ip_id
from swarm_server.commands import CMD
from swarm_server.datagram import DDatagram

from .pio import DroneBase


class Gpion(DroneBase):
    """
    Класс Gpion – наследник DroneBase (аналог Pion), реализующий методы управления через UDP.

    Не запускает MAVLink-соединение и message handler'ы.
    """

    def __init__(
        self,
        ip: str,
        mavlink_port: int,
        name: str,
        dt: float,
        start_from_init: bool = True,
        **kwargs,
    ):
        DroneBase.__init__(
            self,
            ip=ip,
            mavlink_port=mavlink_port,
            name=name,
            mass=0.3,
            position=None,
            attitude=None,
            count_of_checking_points=20,
            logger=False,
            checking_components=True,
            accuracy=0.05,
            dt=dt,
            max_speed=2.0,
        )
        # Настраиваем UDP-сокет для отправки команд
        self.udp_port = CMD.UDP_PORT
        self.target_id: int = extract_ip_id(self.ip)
        if start_from_init:
            self.udp_socket = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
            )
            # Опция BROADCAST для универсальности
            self.udp_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_BROADCAST, 1
            )
            print(f"Gpion создан для {name} с IP {ip}")

    # Реализация методов управления, переопределенных для отправки UDP-команд через protobuf
    def send_speed(
        self, vx: float, vy: float, vz: float, yaw_rate: float
    ) -> None:
        """
        Реализация метода Pion.send_speed: отправка вектора скорости на дрон по broadcast

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
        dtg = DDatagram()
        dtg.command = CMD.SET_SPEED
        dtg.data = [vx, vy, vz, yaw_rate]
        dtg.target_id = self.target_id
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(
            f"UDP команда set_speed отправлена на {self.ip}: {vx}, {vy}, {vz}, {yaw_rate}"
        )

    def send_package(self, command: CMD, data: Optional[list] = None):
        dtg = DDatagram()
        dtg.command = command.value
        dtg.data = data or []
        dtg.target_id = self.target_id
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда {CMD.name} отправлена на {self.ip}")

    def goto(self, x: float, y: float, z: float, yaw: float) -> None:
        point = [x, y, z, yaw]
        self.send_package(CMD.GOTO, point)
        print(f"Точка {x}, {y}, {z}, {yaw}")

    def takeoff(self) -> None:
        self.send_package(CMD.TAKEOFF, [])
        print(f"UDP команда takeoff отправлена на {self.ip}")

    def land(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD.LAND
        dtg.data = []
        dtg.target_id = self.target_id
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда land отправлена на {self.ip}")

    def arm(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD.ARM
        dtg.data = []
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда arm отправлена на {self.ip}")

    def disarm(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD.DISARM
        dtg.data = []
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда disarm отправлена на {self.ip}")

    # Реализация абстрактных методов из Pio
    def set_v(self):
        # Заглушка
        pass

    def stop(self):
        try:
            self.udp_socket.close()
            print("Метод stop вызван: UDP-сокет закрыт.")
        except Exception as e:
            print("Ошибка при закрытии UDP-сокета:", e)

    def _send_heartbeat(self):
        # Заглушка
        pass
