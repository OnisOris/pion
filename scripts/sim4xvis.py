import json
import os
import threading

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Импортируем классы симуляции
from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw


def load_drone_config(config_file: str = "drones_config.json"):
    if os.path.exists(config_file):
        with open(config_file, "r") as f:
            return json.load(f)
    return {}


def save_drone_config(config: dict, config_file: str = "drones_config.json"):
    with open(config_file, "w") as f:
        json.dump(config, f, indent=2)


class SwarmVisualizer:
    """
    Визуализатор роя с четырьмя видами: сверху, спереди, сбоку и 3D
    """

    def __init__(self, sim: SwarmSim, bound: float = 5.0, update_ms: int = 50):
        self.sim = sim
        self.n = sim.n_objects
        self.bound = bound
        self.update_ms = update_ms
        self.colors = np.random.rand(self.n, 3)
        # Создаем фигуру и оси
        self.fig = plt.figure(figsize=(12, 10))
        self.ax_top = self.fig.add_subplot(221)
        self.ax_front = self.fig.add_subplot(222)
        self.ax_side = self.fig.add_subplot(223)
        self.ax_3d = self.fig.add_subplot(224, projection="3d")
        self._setup_axes()
        # Начальные точки
        states = self.sim.get_states()
        xs, ys, zs = states[:, 0], states[:, 1], states[:, 2]
        self.scatter_top = self.ax_top.scatter(xs, ys, c=self.colors, s=30)
        self.scatter_front = self.ax_front.scatter(ys, zs, c=self.colors, s=30)
        self.scatter_side = self.ax_side.scatter(xs, zs, c=self.colors, s=30)
        self.scatter_3d = self.ax_3d.scatter(xs, ys, zs, c=self.colors, s=30)

    def _setup_axes(self):
        # Настройка двухмерных видов
        for ax, title, xlabel, ylabel in [
            (self.ax_top, "Top view (XY)", "X", "Y"),
            (self.ax_front, "Front view (YZ)", "Y", "Z"),
            (self.ax_side, "Side view (XZ)", "X", "Z"),
        ]:
            ax.set_title(title)
            ax.set_xlim(-self.bound, self.bound)
            ax.set_ylim(-self.bound, self.bound)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.grid(True)
            ax.set_aspect("equal")
        # Настройка 3D вида
        self.ax_3d.set_title("3D view")
        self.ax_3d.set_xlim(-self.bound, self.bound)
        self.ax_3d.set_ylim(-self.bound, self.bound)
        self.ax_3d.set_zlim(-self.bound, self.bound)
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")
        self.ax_3d.view_init(elev=30, azim=-60)
        self.ax_3d.grid(True)

    def init_anim(self):
        zero2 = np.zeros((self.n, 2))
        self.scatter_top.set_offsets(zero2)
        self.scatter_front.set_offsets(zero2)
        self.scatter_side.set_offsets(zero2)
        self.scatter_3d._offsets3d = ([], [], [])
        return (
            self.scatter_top,
            self.scatter_front,
            self.scatter_side,
            self.scatter_3d,
        )

    def _update(self, frame):
        states = self.sim.get_states()
        xs, ys, zs = states[:, 0], states[:, 1], states[:, 2]
        self.scatter_top.set_offsets(np.c_[xs, ys])
        self.scatter_front.set_offsets(np.c_[ys, zs])
        self.scatter_side.set_offsets(np.c_[xs, zs])
        self.scatter_3d._offsets3d = (xs, ys, zs)
        return (
            self.scatter_top,
            self.scatter_front,
            self.scatter_side,
            self.scatter_3d,
        )

    def run(self):
        # Отключаем blit, т.к. 3D не поддерживает blit
        self.ani = FuncAnimation(
            self.fig,
            self._update,
            init_func=self.init_anim,
            blit=False,
            interval=self.update_ms,
        )
        plt.tight_layout()
        # Блокирующий показ, чтобы отрисовались все четыре вида
        plt.show()

    def stop(self):
        if hasattr(self, "ani"):
            self.ani.event_source.stop()
        plt.close(self.fig)


class ControlConsole:
    """
    Локальная консоль управления симуляцией роя дронов.

    Поддерживает команды по ID, группам и режимам роя.
    """

    def __init__(
        self,
        number_of_drones: int = 16,
        area_bounds: dict = None,
        dt: float = 0.1,
        config_file: str = "drones_config.json",
    ):
        if area_bounds is None:
            area_bounds = {"x": [-5, 5], "y": [-5, 5]}
        self.area_bounds = area_bounds
        side = int(np.sqrt(number_of_drones))
        sim_objs = create_objects_Point_yaw(
            side, area_bounds["x"], area_bounds["y"]
        )
        self.sim = SwarmSim(sim_objs, dt=dt, logger=False)
        self.sim.simulation_turn_on = False

        self.config_file = config_file
        self.groups = load_drone_config(config_file)
        self.armed = {i: False for i in range(self.sim.n_objects)}
        for idx, obj in enumerate(self.sim.simulation_objects):
            obj.id = idx
            obj.group = self.groups.get(str(idx), 0)

        n = self.sim.n_objects
        self.sim.t_position = np.zeros((n, 6))
        self.sim.t_speed = np.zeros((n, 4))

        # Визуализатор
        max_bound = max(self.area_bounds["x"] + self.area_bounds["y"])
        self.visualizer = SwarmVisualizer(
            self.sim, bound=max_bound, update_ms=int(dt * 1000)
        )
        self.sim_thread = None

    def apply_to_targets(self, target: str):
        if target == "all":
            return list(range(self.sim.n_objects))
        if target.startswith("g:"):
            gid = int(target.split(":")[1])
            return [
                i
                for i, o in enumerate(self.sim.simulation_objects)
                if o.group == gid
            ]
        try:
            return [int(target)]
        except ValueError:
            return []

    def start(self):
        if self.sim_thread and self.sim_thread.is_alive():
            print("Симуляция уже запущена.")
            return
        self.sim.simulation_turn_on = True
        self.sim_thread = threading.Thread(
            target=self.sim.start_simulation_while, daemon=True
        )
        self.sim_thread.start()
        print("Симуляция запущена.")
        # Запуск визуализации (блокирует до закрытия окна)
        self.visualizer.run()

    def stop(self):
        self.sim.stop()
        self.visualizer.stop()
        print("Симуляция и визуализация остановлены.")

    def process(self, line: str):
        parts = line.strip().split()
        if not parts:
            return
        cmd0 = parts[0]
        if cmd0 == "start":
            self.start()
            return
        if cmd0 == "stop" and len(parts) == 1:
            self.stop()
            return
        if cmd0 in ["exit", "quit"]:
            self.stop()
            print("Выход.")
            exit(0)

        target, cmd = cmd0, parts[1] if len(parts) > 1 else ""
        ids = self.apply_to_targets(target)
        if not ids:
            print(f"Неизвестная цель '{target}'")
            return

        if cmd == "set_speed" and len(parts) == 6:
            vx, vy, vz, yr = map(float, parts[2:6])
            for i in ids:
                self.sim.t_speed[i] = [vx, vy, vz, yr]
            print(f"set_speed для {ids}")
        elif cmd == "goto" and len(parts) == 6:
            x, y, z, yaw = map(float, parts[2:6])
            for i in ids:
                self.sim.t_position[i, :3] = [x, y, z]
                self.sim.t_position[i, 5] = yaw
            print(f"goto для {ids}")
        elif cmd == "takeoff":
            for i in ids:
                self.sim.t_position[i, 2] = 1.0
            print(f"takeoff для {ids}")
        elif cmd == "land":
            for i in ids:
                self.sim.t_position[i, 2] = 0.0
            print(f"land для {ids}")
        elif cmd == "arm":
            for i in ids:
                self.armed[i] = True
            print(f"arm для {ids}")
        elif cmd == "disarm":
            for i in ids:
                self.armed[i] = False
            print(f"disarm для {ids}")
        elif cmd == "setgroup" and len(parts) == 3:
            g = int(parts[2])
            for i in ids:
                self.sim.simulation_objects[i].group = g
                self.groups[str(i)] = g
            save_drone_config(self.groups, self.config_file)
            print(f"группа {g} для {ids}")
        elif cmd == "updategroups":
            self.groups = load_drone_config(self.config_file)
            for i, o in enumerate(self.sim.simulation_objects):
                o.group = self.groups.get(str(i), 0)
            print("Группы обновлены.")
        elif cmd in ["trp", "swarm_on"]:
            self.sim.swarm_on = True
            print("Swarm mode ON")
        elif cmd in ["stop_swarm", "swarm_off"]:
            self.sim.swarm_on = False
            print("Swarm mode OFF")
        else:
            print(f"Неизвестная команда '{cmd}'")

    def console_loop(self):
        print(
            "Консоль управления запущена. Введите 'start', 'stop', 'exit', или таргет команду."
        )
        while True:
            try:
                line = input("Command> ")
            except (EOFError, KeyboardInterrupt):
                print("\nВыход.")
                break
            if not line:
                continue
            self.process(line)


def main():
    ControlConsole().console_loop()


if __name__ == "__main__":
    main()
