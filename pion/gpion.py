import socket
import time
import paramiko 
from pion.datagram import DDatagram
from pion.pio import DroneBase  
import sys

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
            ssh.connect(ssh_host, username=ssh_user, password=ssh_password, timeout=10)
            transport = ssh.get_transport()
            
            def exec_command(cmd, timeout=15):
                chan = transport.open_session()
                chan.exec_command(cmd)
                exit_code = chan.recv_exit_status()
                stdout = chan.makefile('r', -1).read()
                stderr = chan.makefile_stderr('r', -1).read()
                return exit_code, stdout, stderr
            
            # Проверка существования сервиса
            exit_code, stdout, stderr = exec_command("sudo systemctl list-unit-files | grep pion_server.service")
            
            if exit_code == 0:
                print("Pion server уже установлен и работает")
                return

            print("Начинаем установку Pion server...")
            
            # Создаем директорию
            exit_code, _, _ = exec_command("mkdir -p ~/code/server")
            if exit_code != 0:
                raise Exception("Ошибка создания директории")

            # Установка зависимостей
            exit_code, _, _ = exec_command(
                "sudo curl -sSL https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_linux.sh | sudo bash",
                timeout=60
            )
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей")

            # Скачивание серверного файла
            exit_code, _, _ = exec_command(
                "wget -q https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/pion_server.py -O ~/code/server/pion_server.py",
                timeout=30
            )
            if exit_code != 0:
                raise Exception("Ошибка загрузки pion_server.py")

            # Создание systemd service
            service_content = f'''\
    [Unit]
    Description=Pion Server
    After=network.target

    [Service]
    User={ssh_host}
    WorkingDirectory=/home/{ssh_host}/code/server
    ExecStart=/home/{ssh_host}/code/server/.venv/bin/python /home/{ssh_host}/code/server/pion_server.py
    Restart=always

    [Install]
    WantedBy=multi-user.target
    '''

            exit_code, _, _ = exec_command(
                f"echo '{service_content}' | sudo tee /etc/systemd/system/pion_server.service >/dev/null",
                timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка создания service file")

            # Reload systemd
            exit_code, _, _ = exec_command("sudo systemctl daemon-reload", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка daemon-reload")

            # Включение сервиса
            exit_code, _, _ = exec_command("sudo systemctl enable pion_server", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка включения сервиса")

            # Запуск сервиса
            exit_code, _, _ = exec_command("sudo systemctl start pion_server", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка запуска сервиса")

            print("Pion server успешно установлен и запущен")
            
        except Exception as e:
            print(f"Критическая ошибка: {str(e)}")
            # Вывод дополнительной информации об ошибках
            try:
                _, logs, _ = ssh.exec_command("journalctl -u pion_server -n 20")
                print("Логи сервиса:\n", logs.read().decode())
            except:
                pass
                
        finally:
            ssh.close()

    def check_pion_server_raspb(self, ssh_host: str, ssh_user: str, ssh_password: str) -> None:
        print(f"Подключаемся по SSH к {ssh_host} как {ssh_user}...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(ssh_host, username=ssh_user, password=ssh_password, timeout=10)
            transport = ssh.get_transport()

            def exec_command(cmd, timeout=15):
                print(f"\nВыполняется команда: {cmd}")
                chan = transport.open_session()
                chan.exec_command(cmd)
                exit_code = chan.recv_exit_status()
                stdout = chan.makefile('r', -1).read().strip()
                stderr = chan.makefile_stderr('r', -1).read().strip()
                print(f"Код завершения: {exit_code}")
                if stdout:
                    print(f"stdout:\n{stdout}")
                if stderr:
                    print(f"stderr:\n{stderr}")
                return exit_code, stdout, stderr

            def exec_command_with_retry(cmd, timeout=15, retries=5, delay=5):
                for attempt in range(1, retries + 1):
                    exit_code, stdout, stderr = exec_command(cmd, timeout=timeout)
                    if exit_code == 0:
                        return exit_code, stdout, stderr
                    if "lock" in stderr.lower():
                        print(
                            f"Обнаружена блокировка dpkg (попытка {attempt}/{retries}). Повтор через {delay} секунд...")
                        time.sleep(delay)
                    else:
                        break
                return exit_code, stdout, stderr

            # Проверяем, установлен ли уже сервис pion_server
            exit_code, stdout, stderr = exec_command("sudo systemctl list-unit-files | grep pion_server.service")
            if exit_code == 0 and stdout:
                print("Pion server уже установлен и работает. Логи:")
                print(stdout)
                return

            print("\nНачинаем установку Pion server для Raspberry Pi Zero 2W...")

            # Создаем директорию для сервера
            exit_code, _, _ = exec_command("mkdir -p ~/code/server")
            if exit_code != 0:
                raise Exception("Ошибка создания директории ~/code/server")

            # Обновляем списки пакетов и устанавливаем необходимые apt-зависимости
            exit_code, _, _ = exec_command_with_retry(
                "sudo apt-get update && sudo apt-get install -y python3 python3-pip wget curl",
                timeout=60
            )
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей через apt-get")

            # Устанавливаем зависимости Pion. Скрипт запускается в директории ~/code/server,
            # после чего создается виртуальное окружение с нужными модулями.
            install_command = ("cd ~/code/server && sudo curl -sSL "
                               "https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_linux.sh | sudo bash")
            exit_code, _, _ = exec_command(install_command, timeout=60)
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей Pion")

            # Скачиваем серверный файл
            exit_code, _, _ = exec_command(
                "wget -q https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/pion_server.py -O ~/code/server/pion_server.py",
                timeout=30)
            if exit_code != 0:
                raise Exception("Ошибка загрузки файла pion_server.py")

            # Создаем systemd unit для Pion server с активацией виртуального окружения
            service_content = f'''\
    [Unit]
    Description=Pion Server
    After=network.target

    [Service]
    User={ssh_user}
    WorkingDirectory=/home/{ssh_user}/code/server
    ExecStart=/bin/bash -c 'source /home/{ssh_user}/code/server/.venv/bin/activate && nohup python3 /home/{ssh_user}/code/server/pion_server.py >> /home/{ssh_user}/code/server/pion_server.log 2>&1'
    Restart=always
    StandardOutput=null
    StandardError=null

    [Install]
    WantedBy=multi-user.target
    '''
            exit_code, _, _ = exec_command(
                f"echo '{service_content}' | sudo tee /etc/systemd/system/pion_server.service >/dev/null",
                timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка создания файла сервиса pion_server.service")

            # Перезагружаем конфигурацию systemd
            exit_code, _, _ = exec_command("sudo systemctl daemon-reload", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка перезагрузки демона systemd")

            # Включаем сервис для автозапуска при загрузке
            exit_code, _, _ = exec_command("sudo systemctl enable pion_server", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка включения сервиса pion_server")

            # Запускаем сервис
            exit_code, _, _ = exec_command("sudo systemctl start pion_server", timeout=15)
            if exit_code != 0:
                raise Exception("Ошибка запуска сервиса pion_server")

            print("\nPion server успешно установлен и запущен на Raspberry Pi Zero 2W")

        except Exception as e:
            print(f"\nКритическая ошибка: {str(e)}")
            try:
                print("\nСбор логов сервиса pion_server:")
                _, logs, _ = ssh.exec_command("journalctl -u pion_server -n 20")
                logs_str = logs.read().decode().strip()
                print("Логи сервиса:\n", logs_str)
            except Exception as inner_e:
                print("Не удалось получить логи:", inner_e)

        finally:
            ssh.close()
            print("SSH-соединение закрыто.")

