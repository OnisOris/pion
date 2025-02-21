import socket
import time
import paramiko 
from pion.datagram import DDatagram
from pion.pio import DroneBase  

# Определяем UDP-порт и коды команд
UDP_PORT = 37020
CMD_SET_SPEED = 1
CMD_GOTO      = 2
CMD_TAKEOFF   = 3
CMD_LAND      = 4
CMD_ARM       = 5
CMD_DISARM    = 6

class Gpion(DroneBase):
    """
    Класс Gpion – наследник DroneBase (аналог Pion), реализующий методы управления через UDP.
    Не запускает MAVLink-соединение и message handler'ы.
    """
    def __init__(self, ip: str, mavlink_port: int, name: str, dt: float, **kwargs):
        DroneBase.__init__(self,
                           ip=ip,
                           mavlink_port=mavlink_port,
                           name=name,
                           mass=0.3,
                           dimension=3,
                           position=None,
                           attitude=None,
                           count_of_checking_points=20,
                           logger=False,
                           checking_components=True,
                           accuracy=0.05,
                           dt=dt,
                           max_speed=2.)
        # Настраиваем UDP-сокет для отправки команд
        self.udp_port = UDP_PORT
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Опция BROADCAST для универсальности
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print(f"Gpion создан для {name} с IP {ip}")

    # Реализация методов управления, переопределенных для отправки UDP-команд через protobuf
    def send_speed(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        dtg = DDatagram()
        dtg.command = CMD_SET_SPEED
        dtg.data = [vx, vy, vz, yaw_rate]
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда set_speed отправлена на {self.ip}: {vx}, {vy}, {vz}, {yaw_rate}")

    def goto(self, x: float, y: float, z: float, yaw: float) -> None:
        dtg = DDatagram()
        dtg.command = CMD_GOTO
        dtg.data = [x, y, z, yaw]
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда goto отправлена на {self.ip}: {x}, {y}, {z}, {yaw}")

    def takeoff(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD_TAKEOFF
        dtg.data = []
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда takeoff отправлена на {self.ip}")

    def land(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD_LAND
        dtg.data = []
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда land отправлена на {self.ip}")

    def arm(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD_ARM
        dtg.data = []
        dtg.target_ip = self.ip
        serialized = dtg.export_serialized()
        self.udp_socket.sendto(serialized, (self.ip, self.udp_port))
        print(f"UDP команда arm отправлена на {self.ip}")

    def disarm(self) -> None:
        dtg = DDatagram()
        dtg.command = CMD_DISARM
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

    def check_pion_server(self, ssh_host: str, ssh_user: str, ssh_password: str) -> None:
        print(f"Подключаемся по SSH к {ssh_host} как {ssh_user}...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(ssh_host, username=ssh_user, password=ssh_password)
            stdin, stdout, stderr = ssh.exec_command("pgrep -f pion_server.py")
            result = stdout.read().decode().strip()
            if result:
                print("Pion server уже запущен на устройстве.")
            else:
                print("Pion server не запущен. Запускаем установку и запуск сервиса...")
                ssh.exec_command("mkdir -p ~/cope/server")
                time.sleep(1)
                ssh.exec_command("sudo curl -sSL https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_linux.sh | sudo bash")
                time.sleep(5)
                ssh.exec_command("wget https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/pion_server.py -O ~/cope/server/pion_server.py")
                time.sleep(2)
                ssh.exec_command("cd ~/cope/server && .venv/bin/python pion_server.py &")
                print("Pion server установлен и запущен.")
            ssh.close()
        except Exception as e:
            print(f"Ошибка при SSH-подключении: {e}")

