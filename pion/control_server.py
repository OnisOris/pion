import socket
from .datagram import DDatagram
import readline
import atexit
import os
from .server import UDPBroadcastClient, UDPBroadcastServer
from queue import Queue
from .commands import *
                
history_file = os.path.join(os.path.expanduser("~"), ".my_console_history")

if os.path.exists(history_file):
    readline.read_history_file(history_file)

atexit.register(readline.write_history_file, history_file)


def get_local_ip():
    """
    Определяет локальный IP-адрес, используя временный UDP-сокет.
    При неудаче возвращает '127.0.0.1'.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Подключаемся к внешнему адресу (8.8.8.8) для определения локального IP
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "127.0.0.1"
    finally:
        s.close()
    return local_ip

#####################################
# Control Server (Консольное приложение)
#####################################

class ControlServer:
    """
    Консольное приложение для отправки команд дронам.
    Синтаксис команд:
      [target] command [параметры]
      
    target:
      - "all" – широковещательная рассылка
      - либо уникальный идентификатор (например, "105" или "105-2")
      
    Команды: set_speed, goto, takeoff, land, arm, disarm, smart_goto, led
    """
    def __init__(self, broadcast_port: int = 37020):
        self.client = UDPBroadcastClient(port=broadcast_port, unique_id="control_server") 
        self.broadcast_port = broadcast_port
        # Здесь network_prefix больше не используется, поскольку target задается как уникальный id
        self.receive_queue = Queue()

        print("Управляющая консоль запущена. Используйте 'all' или уникальный id (например, 105 или 105-2) в качестве target.")


    def send_command(self, command: int, data: list, target: str = "<broadcast>") -> None:
        dt = DDatagram()
        dt.command = command
        dt.data = data
        if target != "<broadcast>":
            # Передаём target_id, чтобы команда адресовалась конкретному экземпляру
            dt.target_id = target
        serialized = dt.export_serialized()
        self.client.socket.sendto(serialized, ("<broadcast>", self.broadcast_port))
        print(f"Команда {command} с данными {data} отправлена для {target}.")

    def console_loop(self):
        print("Запущен консольный интерфейс управления.")
        print("Синтаксис команд: [target] command [параметры]")
        print("  target: 'all' или уникальный id (например, 105 или 105-2)")
        print("  Команды: set_speed, goto, takeoff, land, arm, disarm, smart_goto, led")
        while True:
            try:
                line = input("Command> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nВыход из консоли.")
                break
            if not line:
                continue
            parts = line.split()
            if parts[0].lower() in ["exit", "quit"]:
                print("Выход из консоли.")
                break

            target_part = parts[0]
            target = "<broadcast>" if target_part.lower() == "all" else target_part

            if len(parts) < 2:
                print("Не указана команда.")
                continue

            cmd = parts[1].lower()
            if cmd == "set_speed":
                if len(parts) != 6:
                    print("Использование: [target] set_speed vx vy vz yaw_rate")
                    continue
                try:
                    vx, vy, vz, yaw_rate = map(float, parts[2:6])
                    self.send_command(CMD_SET_SPEED, [vx, vy, vz, yaw_rate], target)
                except ValueError:
                    print("Неверные параметры для set_speed")
            elif cmd == "goto":
                if len(parts) != 6:
                    print("Использование: [target] goto x y z yaw")
                    continue
                try:
                    x, y, z, yaw = map(float, parts[2:6])
                    self.send_command(CMD_GOTO, [x, y, z, yaw], target)
                except ValueError:
                    print("Неверные параметры для goto")
            elif cmd == "takeoff":
                self.send_command(CMD_TAKEOFF, [], target)
            elif cmd == "land":
                self.send_command(CMD_LAND, [], target)
            elif cmd == "arm":
                self.send_command(CMD_ARM, [], target)
            elif cmd == "trp":
                self.send_command(CMD_SWARM_ON, [], target)
            elif cmd == "stop":
                self.send_command(CMD_STOP, [], target)
            elif cmd == "disarm":
                self.send_command(CMD_DISARM, [], target)
            elif cmd == "smart_goto":
                if len(parts) != 6:
                    print("Использование: [target] smart_goto x y z yaw")
                    continue
                try:
                    x, y, z, yaw = map(float, parts[2:6])
                    self.send_command(CMD_SMART_GOTO, [x, y, z, yaw], target)
                except ValueError:
                    print("Неверные параметры для smart_goto")
            elif cmd == "led":
                if len(parts) != 6:
                    print("Использование: [target] led led_id r g b")
                    continue
                try:
                    led_id = int(parts[2])
                    r, g, b = map(int, parts[3:6])
                    self.send_command(CMD_LED, [led_id, r, g, b], target)
                    print(f"Команда LED отправлена: led_id={led_id}, r={r}, g={g}, b={b}")
                except ValueError:
                    print("Неверные параметры для led")
            else:
                print("Неизвестная команда. Доступны: set_speed, goto, takeoff, land, arm, disarm, smart_goto, led")
