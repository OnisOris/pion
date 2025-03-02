import socket
import time
from .datagram import DDatagram
import readline
import atexit
import os

history_file = os.path.join(os.path.expanduser("~"), ".my_console_history")

if os.path.exists(history_file):
    readline.read_history_file(history_file)

atexit.register(readline.write_history_file, history_file)

# Определим коды команд
CMD_SET_SPEED = 1
CMD_GOTO      = 2
CMD_TAKEOFF   = 3
CMD_LAND      = 4
CMD_ARM       = 5
CMD_DISARM    = 6
CMD_SMART_GOTO = 7
CMD_LED       = 8

class UDPBroadcastClient:
    """
    Клиент для отправки UDP сообщений.
    """
    def __init__(self, port: int = 37020) -> None:
        self.port = port
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            # Разрешаем широковещательную рассылку
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            print("UDPBroadcastClient запущен")
        except Exception as error:
            print("Ошибка инициализации UDPBroadcastClient:", error)


class ControlServer:
    """
    Консольное приложение для отправки команд дронам.
    Синтаксис команд:
      [target] command [параметры]
      
    target:
      - "all" – отправить команду всем дронам (широковещательно)
      - либо номер устройства (например, 3 для IP 10.1.100.3)
      
    Команды:
      set_speed vx vy vz yaw_rate
      goto x y z yaw
      takeoff
      land
      arm
      disarm
    """
    def __init__(self, broadcast_port: int = 37020):
        self.broadcast_port = broadcast_port
        self.client = UDPBroadcastClient(port=broadcast_port)
    
    def send_command(self, command: int, data: list, target: str = "<broadcast>") -> None:
        dt = DDatagram()
        dt.command = command
        dt.data = data
        # Если target не широковещательный, добавляем target_ip в datagram:
        if target != "<broadcast>":
            dt.target_ip = target  # новое поле в протобафе
        serialized = dt.export_serialized()
        self.client.socket.sendto(serialized, ("<broadcast>", self.broadcast_port))
        print(f"Команда {command} с данными {data} отправлена для {target}.")



    def console_loop(self):
        print("Запущен консольный интерфейс управления.")
        print("Синтаксис команд: [target] command [параметры]")
        print("  target: 'all' или номер устройства (например, 3 для IP 10.1.100.3)")
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

            # Определяем target
            target_part = parts[0]
            if target_part.lower() == "all":
                target = "<broadcast>"
            else:
                try:
                    num = int(target_part)
                    target = f"10.1.100.{num}"
                except ValueError:
                    print("Неверное значение target. Используйте 'all' или номер устройства (например, 3).")
                    continue

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

