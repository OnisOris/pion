import socket
import time
from typing import Optional

import paramiko

from swarm_server.commands import CMD
from swarm_server.datagram import DDatagram

from pionfunc.functions import extract_ip_id
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
            dimension=3,
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

    def check_pion_server(
        self, ssh_host: str, ssh_user: str, ssh_password: str
    ) -> None:
        print(f"Подключаемся по SSH к {ssh_host} как {ssh_user}...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(
                ssh_host, username=ssh_user, password=ssh_password, timeout=10
            )
            transport = ssh.get_transport()

            def exec_command(cmd, timeout=15):
                chan = transport.open_session()
                chan.exec_command(cmd)
                exit_code = chan.recv_exit_status()
                stdout = chan.makefile("r", -1).read()
                stderr = chan.makefile_stderr("r", -1).read()
                return exit_code, stdout, stderr

            # Проверка существования сервиса
            exit_code, stdout, stderr = exec_command(
                "sudo systemctl list-unit-files | grep pion_server.service"
            )

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
                timeout=60,
            )
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей")

            # Скачивание серверного файла
            exit_code, _, _ = exec_command(
                "wget -q https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/pion_server.py -O ~/code/server/pion_server.py",
                timeout=30,
            )
            if exit_code != 0:
                raise Exception("Ошибка загрузки pion_server.py")

            # Создание systemd service
            service_content = f"""\
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
    """

            exit_code, _, _ = exec_command(
                f"echo '{service_content}' | sudo tee /etc/systemd/system/pion_server.service >/dev/null",
                timeout=15,
            )
            if exit_code != 0:
                raise Exception("Ошибка создания service file")

            # Reload systemd
            exit_code, _, _ = exec_command(
                "sudo systemctl daemon-reload", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка daemon-reload")

            # Включение сервиса
            exit_code, _, _ = exec_command(
                "sudo systemctl enable pion_server", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка включения сервиса")

            # Запуск сервиса
            exit_code, _, _ = exec_command(
                "sudo systemctl start pion_server", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка запуска сервиса")

            print("Pion server успешно установлен и запущен")

        except Exception as e:
            print(f"Критическая ошибка: {str(e)}")
            # Вывод дополнительной информации об ошибках
            try:
                _, logs, _ = ssh.exec_command(
                    "journalctl -u pion_server -n 20"
                )
                print("Логи сервиса:\n", logs.read().decode())
            except Exception as e:
                print(e)

        finally:
            ssh.close()

    def check_pion_server_raspb(
        self, ssh_host: str, ssh_user: str, ssh_password: str
    ) -> None:
        print(f"Подключаемся по SSH к {ssh_host} как {ssh_user}...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(
                ssh_host, username=ssh_user, password=ssh_password, timeout=10
            )
            transport = ssh.get_transport()

            def exec_command(cmd, timeout=15):
                print(f"\nВыполняется команда: {cmd}")
                chan = transport.open_session()
                chan.exec_command(cmd)
                exit_code = chan.recv_exit_status()
                stdout = chan.makefile("r", -1).read().strip()
                stderr = chan.makefile_stderr("r", -1).read().strip()
                print(f"Код завершения: {exit_code}")
                if stdout:
                    print(f"stdout:\n{stdout}")
                if stderr:
                    print(f"stderr:\n{stderr}")
                return exit_code, stdout, stderr

            def exec_command_with_retry(cmd, timeout=15, retries=5, delay=5):
                for attempt in range(1, retries + 1):
                    exit_code, stdout, stderr = exec_command(
                        cmd, timeout=timeout
                    )
                    if exit_code == 0:
                        return exit_code, stdout, stderr
                    if "lock" in stderr.lower():
                        print(
                            f"Обнаружена блокировка dpkg (попытка {attempt}/{retries}). Повтор через {delay} секунд..."
                        )
                        time.sleep(delay)
                    else:
                        break
                return exit_code, stdout, stderr

            # Если сервис уже существует – выводим статус и завершаем установку
            exit_code, stdout, stderr = exec_command(
                "sudo systemctl list-unit-files | grep pion_server.service"
            )
            if exit_code == 0 and stdout:
                print("Pion server уже установлен. Статус сервиса:")
                exit_code, status_stdout, _ = exec_command(
                    "sudo systemctl status pion_server.service"
                )
                print(status_stdout)
                return

            print(
                "\nНачинаем установку Pion server для Raspberry Pi Zero 2W..."
            )

            # Создаем директорию для сервера
            exit_code, _, _ = exec_command("mkdir -p ~/code/server")
            if exit_code != 0:
                raise Exception("Ошибка создания директории ~/code/server")

            # Обновляем списки пакетов и устанавливаем apt-зависимости
            exit_code, _, _ = exec_command_with_retry(
                "sudo apt-get update && sudo apt-get install -y python3 python3-pip wget curl",
                timeout=60,
            )
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей через apt-get")

            # Устанавливаем зависимости Pion – запускаем скрипт установки
            install_command = (
                "cd ~/code/server && sudo curl -sSL "
                "https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_linux.sh | sudo bash"
            )
            exit_code, _, _ = exec_command(install_command, timeout=60)
            if exit_code != 0:
                raise Exception("Ошибка установки зависимостей Pion")

            # Скачиваем файл pion_server.py
            exit_code, _, _ = exec_command(
                "wget -q https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/pion_server.py -O ~/code/server/pion_server.py",
                timeout=30,
            )
            if exit_code != 0:
                raise Exception("Ошибка загрузки файла pion_server.py")

            # Создаем systemd unit для Pion server через heredoc
            unit_command = f"""sudo tee /etc/systemd/system/pion_server.service > /dev/null << 'EOF'
    [Unit]
    Description=Pion Server
    After=network.target

    [Service]
    User={ssh_user}
    WorkingDirectory=/home/{ssh_user}/code/server
    ExecStart=/bin/bash -c "source /home/{ssh_user}/code/server/.venv/bin/activate && nohup python3 /home/{ssh_user}/code/server/pion_server.py >> /home/{ssh_user}/code/server/pion_server.log 2>&1"
    Restart=always
    StandardOutput=null
    StandardError=null

    [Install]
    WantedBy=multi-user.target
    EOF
    """
            exit_code, _, _ = exec_command(unit_command, timeout=15)
            if exit_code != 0:
                raise Exception(
                    "Ошибка создания файла сервиса pion_server.service"
                )

            # Перезагружаем конфигурацию systemd
            exit_code, _, _ = exec_command(
                "sudo systemctl daemon-reload", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка перезагрузки демона systemd")

            # Если unit замаскирован, размаскируем его
            exec_command(
                "sudo systemctl unmask pion_server.service", timeout=10
            )

            # Включаем сервис для автозапуска
            exit_code, _, _ = exec_command(
                "sudo systemctl enable pion_server.service", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка включения сервиса pion_server")

            # Запускаем сервис
            exit_code, _, _ = exec_command(
                "sudo systemctl start pion_server.service", timeout=15
            )
            if exit_code != 0:
                raise Exception("Ошибка запуска сервиса pion_server")

            print(
                "\nPion server успешно установлен и запущен на Raspberry Pi Zero 2W"
            )

        except Exception as e:
            print(f"\nКритическая ошибка: {str(e)}")
            try:
                print("\nСбор логов сервиса pion_server:")
                _, logs, _ = ssh.exec_command(
                    "journalctl -u pion_server -n 20"
                )
                logs_str = logs.read().decode().strip()
                print("Логи сервиса:\n", logs_str)
            except Exception as inner_e:
                print("Не удалось получить логи:", inner_e)

        finally:
            ssh.close()
            print("SSH-соединение закрыто.")

    def remove_existing_pion_service(
        self, ssh_host: str, ssh_user: str, ssh_password: str
    ) -> None:
        print(
            f"Подключаемся по SSH к {ssh_host} для удаления сервиса pion_server..."
        )
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(
                ssh_host, username=ssh_user, password=ssh_password, timeout=10
            )
            transport = ssh.get_transport()

            def exec_command(cmd, timeout=15):
                print(f"\nВыполняется команда: {cmd}")
                chan = transport.open_session()
                chan.exec_command(cmd)
                exit_code = chan.recv_exit_status()
                stdout = chan.makefile("r", -1).read().strip()
                stderr = chan.makefile_stderr("r", -1).read().strip()
                print(f"Код завершения: {exit_code}")
                if stdout:
                    print(f"stdout:\n{stdout}")
                if stderr:
                    print(f"stderr:\n{stderr}")
                return exit_code, stdout, stderr

            # Останавливаем сервис, если он запущен
            exec_command("sudo systemctl stop pion_server")
            # Отключаем автозапуск сервиса
            exec_command("sudo systemctl disable pion_server")
            # Удаляем файл сервиса
            exec_command("sudo rm -f /etc/systemd/system/pion_server.service")
            # Обновляем конфигурацию systemd
            exec_command("sudo systemctl daemon-reload")

            print(
                "\nСервис pion_server успешно удалён. Теперь можно провести тест установки."
            )

        except Exception as e:
            print(f"\nОшибка при удалении сервиса: {str(e)}")
        finally:
            ssh.close()
            print("SSH-соединение закрыто.")
