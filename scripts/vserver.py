# /// script
# dependencies = [
#   "matplotlib",
#   "pionsdk @ git+https://github.com/OnisOris/pion@dev",
#   "tornado",
#   "pyqt5",
# ]
# ///
import argparse
import socket
import threading
import time
from queue import Queue

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from swarm_server import DDatagram

parser = argparse.ArgumentParser(description="Запуск визуализации")
parser.add_argument("--web", action="store_true", help="Запуск в web")
args = parser.parse_args()
if args.web:
    matplotlib.use("WebAgg")
else:
    matplotlib.use("Qt5Agg")
args = parser.parse_args()


def extract_ip_id(ip: str) -> str:
    """
    Возвращает последний октет IP как строку.

    Если не получается, возвращает хэш в диапазоне [0, 1000).
    """
    print(ip)
    parts = ip.split(".")
    if len(parts) == 4:
        try:
            return parts[-1]
        except ValueError:
            pass
    return str(abs(hash(ip)) % 1000)


class SwarmVisualizer2D:
    def __init__(self, port=37020):
        self.port = port
        self.data_queue = Queue()
        self.drones = {}  # ключи – исходный payload.id, значения – словари с данными дрона
        self.colors = {}
        self.trails_length = 30
        self.running = True
        self.lock = threading.Lock()  # Блокировка для синхронизации доступа

        # Словарь для сопоставления длинных id с короткими метками
        self.id_mapping = {}

        # UDP сервер
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.port))

        # Запуск потока приёма данных
        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

        # Инициализация графика
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_xlim(-5.5, 5.5)
        self.ax.set_ylim(-5.5, 5.5)
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_title("Real-time 2D Drone Swarm Visualization")
        self.ax.grid(True)
        self.ax.set_aspect("equal")

    def receive_data(self):
        decoder = DDatagram()
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                valid, payload = decoder.read_serialized(data)
                # Проверяем, что данных достаточно (1 - IP, 3 - позиция, 3 - скорость)
                if valid and len(payload.data) >= 7:
                    self.process_payload(payload, addr)
            except Exception as e:
                print(f"Receive error: {e}")

    def process_payload(self, payload, addr):
        with self.lock:
            # Извлечение ip из данных
            try:
                ip_num = int(payload.data[0])
                ip = socket.inet_ntoa(ip_num.to_bytes(4, byteorder="big"))
            except (OverflowError, IndexError):
                ip = "Invalid IP"

            # Если для данного payload.id ещё не определена короткая метка, вычисляем её:
            if payload.id not in self.id_mapping:
                base = extract_ip_id(ip)
                # Считаем, сколько уже есть меток с таким базовым значением
                duplicates = [
                    v for v in self.id_mapping.values() if v.startswith(base)
                ]
                if duplicates:
                    short_id = f"{base}-{len(duplicates) + 1}"
                else:
                    short_id = base
                self.id_mapping[payload.id] = short_id

            short_id = self.id_mapping[payload.id]

            # Извлекаем позицию: индексы 1, 2, 3
            position = np.array(
                [
                    payload.data[1],  # X
                    payload.data[2],  # Y
                    payload.data[3],  # Z
                ]
            )

            # Извлекаем скорость: индексы 4, 5, 6
            velocity = np.array(
                [
                    payload.data[4],  # Vx
                    payload.data[5],  # Vy
                    payload.data[6],  # Vz
                ]
            )
            print(payload.data)
            attitude = (
                np.array(payload.data[7:13])
                if len(payload.data) >= 7
                else np.zeros(3)
            )
            t_speed = (
                np.array(payload.data[13:17])
                if len(payload.data) >= 14
                else np.zeros(4)
            )
            if payload.id not in self.drones:
                self.colors[payload.id] = np.random.rand(
                    3,
                )
                self.drones[payload.id] = {
                    "position": position,
                    "attitude": attitude,
                    "velocity": velocity,
                    "t_speed": t_speed,
                    "trail": [],
                    "ip": ip,
                    "last_update": time.time(),
                    "short_id": ip_num,
                }
            else:
                self.drones[payload.id]["position"] = position
                self.drones[payload.id]["attitude"] = attitude
                self.drones[payload.id]["velocity"] = velocity
                self.drones[payload.id]["t_speed"] = t_speed
                self.drones[payload.id]["last_update"] = time.time()
                trail = self.drones[payload.id]["trail"]
                trail.append(position.copy()[:2])
                if len(trail) > self.trails_length:
                    trail.pop(0)

    def update_plot(self, frame):
        with self.lock:
            self.ax.clear()
            self.setup_plot()

            current_time = time.time()
            to_delete = []

            # Итерация по дронам
            for drone_key in list(self.drones.keys()):
                data = self.drones[drone_key]

                # Удаляем неактивных дронов (без обновлений >3 сек)
                if current_time - data["last_update"] > 3:
                    to_delete.append(drone_key)
                    continue

                color = self.colors[drone_key]
                pos = data["position"]
                trail = np.array(data["trail"])
                velocity = data["velocity"]

                # Рисуем дрона (точка)
                size = 80 + pos[2] * 5
                self.ax.scatter(
                    pos[0],
                    pos[1],
                    c=[color],
                    s=size,
                    marker="o",
                    edgecolors="k",
                    label=f"Drone {data['short_id']}",
                )

                # Текст с высотой
                self.ax.text(
                    pos[0] + 0.3,
                    pos[1] + 0.3,
                    f"{pos[2]:.1f}m",
                    color=color,
                    fontsize=8,
                )

                # Рисуем траекторию (след)
                if len(trail) > 1:
                    self.ax.plot(
                        trail[:, 0],
                        trail[:, 1],
                        c=color,
                        linestyle=":",
                        alpha=0.6,
                        linewidth=1,
                    )

                # Рассчитываем угол (yaw) из вектора скорости (если ненулевой)
                # if np.linalg.norm(velocity[:2]) > 0.001:
                #     yaw = np.arctan2(velocity[1], velocity[0])
                # else:
                #     yaw = 0
                yaw = data["attitude"][2]
                t_speed = data["t_speed"]
                # Рисуем стрелку направления (yaw) с фиксированной длиной
                self.draw_orientation(pos[:2], yaw, color)
                self.draw_t_speed_vector(pos[:2], t_speed, "red")
                # Рисуем вектор скорости (масштабированный по модулю)
                self.draw_velocity_vector(pos[:2], velocity, color)

            # Удаление неактивных дронов
            for did in to_delete:
                del self.drones[did]
                del self.colors[did]
                # Не удаляем соответствующую запись из id_mapping, чтобы при повторном появлении тот же short_id использоваться было согласованно

            if self.drones:
                self.ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))

        return self.ax

    def draw_orientation(self, position, yaw, color):
        print("draw_orientation, yaw = ", yaw)
        arrow_length = 1.2  # фиксированная длина стрелки направления
        dx = arrow_length * np.cos(yaw + np.pi / 2)
        dy = arrow_length * np.sin(yaw + np.pi / 2)
        self.ax.quiver(
            position[0],
            position[1],
            dx,
            dy,
            color=color,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.003,
            headwidth=4,
            headlength=5,
        )

    def draw_velocity_vector(self, position, velocity, color):
        factor = 10  # масштабирование вектора скорости
        arrow_dx = velocity[0] * factor
        arrow_dy = velocity[1] * factor
        self.ax.quiver(
            position[0],
            position[1],
            arrow_dx,
            arrow_dy,
            color=color,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.005,
            headwidth=5,
            headlength=7,
            alpha=0.8,
        )

    def draw_t_speed_vector(self, position, t_speed, color):
        factor = 1  # масштабирование вектора скорости
        arrow_dx = t_speed[0] * factor
        arrow_dy = t_speed[1] * factor
        self.ax.quiver(
            position[0],
            position[1],
            arrow_dx,
            arrow_dy,
            color=color,
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.005,
            headwidth=5,
            headlength=7,
            alpha=0.8,
        )

    def run(self):
        any = FuncAnimation(
            self.fig, self.update_plot, interval=500, cache_frame_data=False
        )
        plt.show()
        return any

    def shutdown(self):
        self.running = False
        self.sock.close()
        plt.close("all")


if __name__ == "__main__":
    visualizer = SwarmVisualizer2D(port=37020)
    try:
        visualizer.run()
    except KeyboardInterrupt:
        visualizer.shutdown()
    finally:
        visualizer.shutdown()
