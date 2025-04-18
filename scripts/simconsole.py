# /// script
# dependencies = [
#   "matplotlib",
#   "pionsdk @ git+https://github.com/OnisOris/pion@dev",
#   "tornado",
# ]
# ///
import threading
import json
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Импорт ваших классов симуляции
from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw

def load_drone_config(config_file: str = "drones_config.json") -> dict:
    if os.path.exists(config_file):
        with open(config_file, "r") as f:
            return json.load(f)
    return {}

def save_drone_config(config: dict, config_file: str = "drones_config.json") -> None:
    with open(config_file, "w") as f:
        json.dump(config, f, indent=2)

class SwarmVisualizer:
    """
    Визуализатор роя: 4 вида — сверху (XY), спереди (YZ), сбоку (XZ) и 3D.
    Анимация создаётся в конструкторе; окно открывается в run().
    """
    def __init__(self, sim: SwarmSim, bound: float = 5.0, interval_ms: int = 50):
        self.sim = sim
        self.n = sim.n_objects
        self.bound = bound
        self.interval = interval_ms
        self.colors = np.random.rand(self.n, 3)

        # Настройка фигур и осей
        self.fig = plt.figure(figsize=(12, 10))
        self.ax_top   = self.fig.add_subplot(221)
        self.ax_front = self.fig.add_subplot(222)
        self.ax_side  = self.fig.add_subplot(223)
        self.ax_3d    = self.fig.add_subplot(224, projection='3d')
        self._setup_axes()

        # Первичное размещение точек
        xs, ys, zs = self.sim.get_states().T[:3]
        self.scatter_top   = self.ax_top.scatter(xs, ys, c=self.colors, s=30)
        self.scatter_front = self.ax_front.scatter(ys, zs, c=self.colors, s=30)
        self.scatter_side  = self.ax_side.scatter(xs, zs, c=self.colors, s=30)
        self.scatter_3d    = self.ax_3d.scatter(xs, ys, zs, c=self.colors, s=30)

        # Инициализация FuncAnimation (не вызывает show)
        self.anim = FuncAnimation(
            self.fig,
            self._update,
            init_func=self._init_anim,
            blit=False,
            interval=self.interval,
            cache_frame_data=False
        )

    def _setup_axes(self):
        for ax, title, xlabel, ylabel in [
            (self.ax_top, 'Top (XY)', 'X', 'Y'),
            (self.ax_front, 'Front (YZ)', 'Y', 'Z'),
            (self.ax_side, 'Side (XZ)', 'X', 'Z')]:
            ax.set_title(title)
            ax.set_xlim(-self.bound, self.bound)
            ax.set_ylim(-self.bound, self.bound)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect('equal')
            ax.grid(True)
        self.ax_3d.set_title('3D')
        self.ax_3d.set_xlim(-self.bound, self.bound)
        self.ax_3d.set_ylim(-self.bound, self.bound)
        self.ax_3d.set_zlim(-self.bound, self.bound)
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.view_init(elev=30, azim=-60)
        self.ax_3d.grid(True)

    def _init_anim(self):
        zero2 = np.zeros((self.n, 2))
        self.scatter_top.set_offsets(zero2)
        self.scatter_front.set_offsets(zero2)
        self.scatter_side.set_offsets(zero2)
        self.scatter_3d._offsets3d = ([], [], [])
        return self.scatter_top, self.scatter_front, self.scatter_side, self.scatter_3d

    def _update(self, frame):
        # Обновляем по состоянию симуляции
        xs, ys, zs = self.sim.get_states().T[:3]
        self.scatter_top.set_offsets(np.column_stack([xs, ys]))
        self.scatter_front.set_offsets(np.column_stack([ys, zs]))
        self.scatter_side.set_offsets(np.column_stack([xs, zs]))
        self.scatter_3d._offsets3d = (xs, ys, zs)
        return self.scatter_top, self.scatter_front, self.scatter_side, self.scatter_3d

    def run(self):
        """Открывает окно анимации (должно вызываться из главного потока)."""
        plt.tight_layout()
        plt.show()

    def stop(self):
        plt.close(self.fig)

class ControlConsole:
    """
    Консоль управления роем дронов.
    Команды: start, stop, exit, и команды по ID или группам.
    """
    def __init__(self, number_of_drones=16, area_bounds=None, dt=0.1, config_file="drones_config.json"):
        if area_bounds is None:
            area_bounds = {'x': [-15, 15], 'y': [-15, 15]}
        # Инициализация симуляции
        side = int(np.sqrt(number_of_drones))
        sim_objs = create_objects_Point_yaw(side, area_bounds['x'], area_bounds['y'])
        self.sim = SwarmSim(sim_objs, dt=dt, logger=True)
        self.sim.simulation_turn_on = False

        # Загрузка конфигурации арминга/групп
        self.config_file = config_file
        self.groups = load_drone_config(config_file)
        self.armed = {i: False for i in range(self.sim.n_objects)}
        for i, obj in enumerate(self.sim.simulation_objects):
            obj.id = i
            obj.group = self.groups.get(str(i), 0)

        # Инициализация таргетов
        n = self.sim.n_objects
        self.sim.t_position = np.zeros((n, 6))
        self.sim.t_speed    = np.zeros((n, 4))

        # Подготовка визуализатора
        bound    = max(area_bounds['x'] + area_bounds['y'])
        interval = int(dt * 1000)
        self.visualizer = SwarmVisualizer(self.sim, bound=bound, interval_ms=interval)
        self.sim_thread  = None

    def apply_to_targets(self, target: str) -> list[int]:
        if target == 'all':
            return list(range(self.sim.n_objects))
        if target.startswith('g:'):
            gid = int(target.split(':')[1])
            return [i for i,o in enumerate(self.sim.simulation_objects) if o.group == gid]
        try:
            return [int(target)]
        except ValueError:
            return []

    def start(self):
        if self.sim.simulation_turn_on:
            print("Симуляция уже запущена.")
            return
        # Запускаем фоновой поток симуляции
        self.sim.simulation_turn_on = True
        self.sim_thread = threading.Thread(
            target=self.sim.start_simulation_while,
            daemon=True
        )
        self.sim_thread.start()
        print("Симуляция запущена.")

    def stop(self):
        if not self.sim.simulation_turn_on:
            print("Симуляция не запущена.")
            return
        self.sim.simulation_turn_on = False
        if self.sim_thread:
            self.sim_thread.join(timeout=1)
        print("Симуляция остановлена.")

    def process(self, line: str):
        parts = line.strip().split()
        if not parts:
            return
        cmd0 = parts[0].lower()
        if cmd0 == 'start':
            return self.start()
        if cmd0 == 'stop' and len(parts) == 1:
            return self.stop()
        if cmd0 in ['exit', 'quit']:
            self.stop(); print("Выход."); exit(0)
        ids = self.apply_to_targets(cmd0)
        if not ids:
            print(f"Неизвестная цель '{cmd0}'"); return
        cmd  = parts[1].lower() if len(parts) > 1 else ''
        args = parts[2:]
        # Добавьте здесь обработку команд: arm, takeoff, land, goto и т.д.
        print(f"Команда '{cmd}' применена к: {ids}")

    def console_loop(self):
        print("Консоль управления запущена. Введите 'start', 'stop', 'exit' или команды для дронов.")
        while True:
            try:
                line = input("Command> ")
            except (EOFError, KeyboardInterrupt):
                print("\nВыход."); break
            if line.strip():
                self.process(line)

if __name__ == "__main__":
    console = ControlConsole()
    console.start()  # <<<--- запускаем симуляцию перед визуализацией
    threading.Thread(target=console.console_loop, daemon=True).start()
    console.visualizer.run()
