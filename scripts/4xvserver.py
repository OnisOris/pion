# /// script
# dependencies = [
#   "matplotlib",
#   "pionsdk @ git+https://github.com/OnisOris/pion@dev",
#   "tornado",
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


class SwarmVisualizer4D:
    def __init__(self, port=37020):
        self.port = port
        self.data_queue = Queue()
        self.drones = {}
        self.colors = {}
        self.trails_length = 30
        self.running = True
        self.lock = threading.Lock()

        self.id_mapping = {}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.port))

        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

        self.fig = plt.figure(figsize=(12, 10))
        self.ax_top = self.fig.add_subplot(221)  # Верхний вид (XY)
        self.ax_front = self.fig.add_subplot(222)  # Вид спереди (YZ)
        self.ax_side = self.fig.add_subplot(223)  # Боковой вид (XZ)
        self.ax_3d = self.fig.add_subplot(224, projection="3d")  # 3D вид
        self.ax_3d.set_navigate(False)
        self.ax_3d.mouse_init = lambda: None

        self.setup_axes()

    def setup_axes(self):
        # Верхний вид (XY)
        self.ax_top.set_title("Top view (XY)")
        self.ax_top.set_xlabel("X")
        self.ax_top.set_ylabel("Y")
        self.ax_top.set_xlim(-5.5, 5.5)
        self.ax_top.set_ylim(-5.5, 5.5)
        self.ax_top.grid(True)
        # Вид спереди (YZ)
        self.ax_front.set_title("Front view (YZ)")
        self.ax_front.set_xlabel("Y")
        self.ax_front.set_ylabel("Z")
        self.ax_front.set_xlim(-5.5, 5.5)
        self.ax_front.set_ylim(-5.5, 5.5)
        self.ax_front.grid(True)
        # Боковой вид (XZ)
        self.ax_side.set_title("Side view (XZ)")
        self.ax_side.set_xlabel("X")
        self.ax_side.set_ylabel("Z")
        self.ax_side.set_xlim(-5.5, 5.5)
        self.ax_side.set_ylim(-5.5, 5.5)
        self.ax_side.grid(True)
        # 3D вид
        self.ax_3d.set_title("3D view")
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")
        self.ax_3d.set_xlim(-5.5, 5.5)
        self.ax_3d.set_ylim(-5.5, 5.5)
        self.ax_3d.set_zlim(-5.5, 5.5)
        self.ax_3d.grid(True)
        self.ax_3d.view_init(elev=30, azim=-60)

    def receive_data(self):
        decoder = DDatagram()
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                valid, payload = decoder.read_serialized(data)
                if valid and len(payload.data) >= 7:
                    self.process_payload(payload, addr)
            except Exception as e:
                print(f"Receive error: {e}")

    def process_payload(self, payload, addr):
        with self.lock:
            try:
                id_value = int(payload.data[0])
                print(id_value)
            except Exception:
                id_value = "Invalid ID"

            if payload.id not in self.id_mapping:
                self.id_mapping[payload.id] = str(id_value)
            short_id = self.id_mapping[payload.id]

            position = np.array(
                [
                    payload.data[1],  # X
                    payload.data[2],  # Y
                    payload.data[3],  # Z
                ]
            )
            velocity = np.array(
                [
                    payload.data[4],
                    payload.data[5],
                    payload.data[6],
                ]
            )
            attitude = (
                np.array(payload.data[7:13])
                if len(payload.data) >= 13
                else np.zeros(6)
            )
            t_speed = (
                np.array(payload.data[13:17])
                if len(payload.data) >= 17
                else np.zeros(4)
            )

            if payload.id not in self.drones:
                self.colors[payload.id] = np.random.rand(3)
                self.drones[payload.id] = {
                    "position": position,
                    "velocity": velocity,
                    "attitude": attitude,
                    "t_speed": t_speed,
                    "trail": [position.copy()],
                    "short_id": short_id,
                    "last_update": time.time(),
                }
            else:
                self.drones[payload.id]["position"] = position
                self.drones[payload.id]["velocity"] = velocity
                self.drones[payload.id]["attitude"] = attitude
                self.drones[payload.id]["t_speed"] = t_speed
                self.drones[payload.id]["last_update"] = time.time()
                self.drones[payload.id]["trail"].append(position.copy())
                if len(self.drones[payload.id]["trail"]) > self.trails_length:
                    self.drones[payload.id]["trail"].pop(0)

    def draw_vectors_2d(self, ax, pos, velocity, factor=0.5, color="red"):
        arrow_dx = velocity[0] * factor
        arrow_dy = velocity[1] * factor
        ax.quiver(
            pos[0],
            pos[1],
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

    def draw_vectors_3d(self, ax, pos, velocity, factor=0.5, color="red"):
        ax.quiver(
            pos[0],
            pos[1],
            pos[2],
            velocity[0] * factor,
            velocity[1] * factor,
            velocity[2] * factor,
            color=color,
            length=1.0,
            normalize=False,
        )

    def update_plot(self, frame):
        with self.lock:
            for ax in [self.ax_top, self.ax_front, self.ax_side]:
                ax.cla()
            self.ax_3d.cla()
            self.setup_axes()

            current_time = time.time()
            to_delete = []

            for drone_key in list(self.drones.keys()):
                data = self.drones[drone_key]
                if current_time - data["last_update"] > 3:
                    to_delete.append(drone_key)
                    continue

                color = self.colors[drone_key]
                pos = data["position"]
                velocity = data["velocity"]

                self.ax_top.scatter(
                    pos[0],
                    pos[1],
                    c=[color],
                    s=80,
                    marker="o",
                    edgecolors="k",
                    label=data["short_id"],
                )
                if len(data["trail"]) > 1:
                    trail = np.array(data["trail"])
                    self.ax_top.plot(
                        trail[:, 0],
                        trail[:, 1],
                        c=color,
                        linestyle=":",
                        alpha=0.6,
                        linewidth=1,
                    )
                self.draw_vectors_2d(
                    self.ax_top, pos, velocity, factor=0.2, color="red"
                )

                self.ax_front.scatter(
                    pos[1],
                    pos[2],
                    c=[color],
                    s=80,
                    marker="o",
                    edgecolors="k",
                    label=data["short_id"],
                )
                if len(data["trail"]) > 1:
                    trail = np.array(data["trail"])
                    self.ax_front.plot(
                        trail[:, 1],
                        trail[:, 2],
                        c=color,
                        linestyle=":",
                        alpha=0.6,
                        linewidth=1,
                    )
                vel_front = np.array([velocity[1], velocity[2]])
                self.ax_front.quiver(
                    pos[1],
                    pos[2],
                    vel_front[0] * 0.2,
                    vel_front[1] * 0.2,
                    color="red",
                    angles="xy",
                    scale_units="xy",
                    scale=1,
                    width=0.005,
                    headwidth=5,
                    headlength=7,
                    alpha=0.8,
                )

                self.ax_side.scatter(
                    pos[0],
                    pos[2],
                    c=[color],
                    s=80,
                    marker="o",
                    edgecolors="k",
                    label=data["short_id"],
                )
                if len(data["trail"]) > 1:
                    trail = np.array(data["trail"])
                    self.ax_side.plot(
                        trail[:, 0],
                        trail[:, 2],
                        c=color,
                        linestyle=":",
                        alpha=0.6,
                        linewidth=1,
                    )
                vel_side = np.array([velocity[0], velocity[2]])
                self.ax_side.quiver(
                    pos[0],
                    pos[2],
                    vel_side[0] * 0.2,
                    vel_side[1] * 0.2,
                    color="red",
                    angles="xy",
                    scale_units="xy",
                    scale=1,
                    width=0.005,
                    headwidth=5,
                    headlength=7,
                    alpha=0.8,
                )

                # 3D view
                self.ax_3d.scatter(
                    pos[0],
                    pos[1],
                    pos[2],
                    c=[color],
                    s=80,
                    marker="o",
                    edgecolors="k",
                    label=data["short_id"],
                )
                if len(data["trail"]) > 1:
                    trail = np.array(data["trail"])
                    self.ax_3d.plot(
                        trail[:, 0],
                        trail[:, 1],
                        trail[:, 2],
                        c=color,
                        linestyle=":",
                        alpha=0.6,
                        linewidth=1,
                    )
                self.draw_vectors_3d(
                    self.ax_3d, pos, velocity, factor=0.2, color="red"
                )

            for did in to_delete:
                del self.drones[did]
                del self.colors[did]

            for ax in [self.ax_top, self.ax_front, self.ax_side]:
                ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))

        return self.ax_top, self.ax_front, self.ax_side, self.ax_3d

    def run(self):
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=500, cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()

    def shutdown(self):
        self.running = False
        self.sock.close()
        plt.close("all")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Запуск 4D визуализации дронов"
    )
    parser.add_argument(
        "--web", action="store_true", help="Запуск в web режиме"
    )
    args = parser.parse_args()
    if args.web:
        matplotlib.use("WebAgg")

    visualizer = SwarmVisualizer4D(port=37020)
    try:
        visualizer.run()
    except KeyboardInterrupt:
        visualizer.shutdown()
    finally:
        visualizer.shutdown()
