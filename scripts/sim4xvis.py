# /// script
# dependencies = [
#   "matplotlib",
#   "pionsdk @ git+https://github.com/OnisOris/pion@dev",
#   "tornado",
# ]
# ///
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Импортируем ваш класс симуляции
from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw


class SwarmVisualizer:
    """
    Упрощённый визуализатор для класса SwarmSim
    """

    def __init__(
        self,
        sim: SwarmSim,
        bound: float = 5.0,
        update_ms: int = 50,
    ):
        self.sim = sim
        self.n = sim.n_objects
        self.bound = bound
        self.update_ms = update_ms

        self.colors = np.random.rand(self.n, 3)

        self.fig = plt.figure(figsize=(12, 10))
        self.ax_top = self.fig.add_subplot(221)
        self.ax_front = self.fig.add_subplot(222)
        self.ax_side = self.fig.add_subplot(223)
        self.ax_3d = self.fig.add_subplot(224, projection="3d")
        self._setup_axes()

        states = self.sim.get_states()
        xs, ys, zs = states[:, 0], states[:, 1], states[:, 2]

        self.scatter_top = self.ax_top.scatter(xs, ys, c=self.colors, s=30)
        self.scatter_front = self.ax_front.scatter(ys, zs, c=self.colors, s=30)
        self.scatter_side = self.ax_side.scatter(xs, zs, c=self.colors, s=30)
        self.scatter_3d = self.ax_3d.scatter(xs, ys, zs, c=self.colors, s=30)

    def _setup_axes(self):
        views = [
            (self.ax_top, "Top view (XY)", "X", "Y"),
            (self.ax_front, "Front view (YZ)", "Y", "Z"),
            (self.ax_side, "Side view (XZ)", "X", "Z"),
        ]
        for ax, title, xlabel, ylabel in views:
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_xlim(-self.bound, self.bound)
            ax.set_ylim(-self.bound, self.bound)
            ax.grid(True)
            ax.set_aspect("equal")
        self.ax_3d.set_title("3D view")
        self.ax_3d.set_xlabel("X")
        self.ax_3d.set_ylabel("Y")
        self.ax_3d.set_zlabel("Z")
        self.ax_3d.set_xlim(-self.bound, self.bound)
        self.ax_3d.set_ylim(-self.bound, self.bound)
        self.ax_3d.set_zlim(-self.bound, self.bound)
        self.ax_3d.view_init(elev=30, azim=-60)
        self.ax_3d.grid(True)

    def init_anim(self):
        """Инициализация для FuncAnimation"""
        zero = np.zeros((self.n, 2))
        self.scatter_top.set_offsets(zero)
        self.scatter_front.set_offsets(zero)
        self.scatter_side.set_offsets(zero)
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
        self.ani = FuncAnimation(
            self.fig,
            self._update,
            init_func=self.init_anim,
            blit=True,
            interval=self.update_ms,
        )
        plt.tight_layout()
        plt.show()


def main():
    num = 16
    params = {
        "kp": np.ones((num, 6)) * 0.2,
        "ki": np.zeros((num, 6)),
        "kd": np.ones((num, 6)),
        "attraction_weight": 1.0,
        "cohesion_weight": 1.0,
        "alignment_weight": 1.0,
        "repulsion_weight": 4.0,
        "unstable_weight": 1.0,
        "noise_weight": 1.0,
        "safety_radius": 0.5,
        "max_acceleration": 10,
        "max_speed": 1,
        "unstable_radius": 1.5,
    }
    points = create_objects_Point_yaw(int(np.sqrt(num)), [-5, 5], [-5, 5])
    sim = SwarmSim(points, dt=0.01, logger=True, params=params)
    sim.start_simulation_while()

    viz = SwarmVisualizer(sim, bound=5.0, update_ms=50)
    try:
        viz.run()
    finally:
        sim.stop()


if __name__ == "__main__":
    main()
