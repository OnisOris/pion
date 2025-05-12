# /// script
# dependencies = [
#   "matplotlib",
#   "numpy",
#   "tornado",
#   "pyqt6",
# ]
# ///
import argparse

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline swarm animation from .npz trajectories"
    )
    parser.add_argument(
        "--file",
        type=str,
        default="swarm_trajectory.npz",
        help="Path to .npz file with arrays 'time' and 'states'",
    )
    parser.add_argument(
        "--backend",
        choices=["web", "qt"],
        default="qt",
        help="Matplotlib backend: 'web' (WebAgg) or 'qt' (QtAgg)",
    )
    parser.add_argument(
        "--width", type=float, default=16, help="Figure width in inches"
    )
    parser.add_argument(
        "--height", type=float, default=9, help="Figure height in inches"
    )
    parser.add_argument(
        "--bound", type=float, default=5, help="Axis limit for X, Y, Z"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Initial speed multiplier (1.0 = realtime)",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.1,
        help="Step of time in simulation",
    )
    parser.add_argument(
        "--decimate",
        type=int,
        default=1,
        help="Keep every Nth frame (reduces data size)",
    )
    parser.add_argument(
        "--max-fps",
        type=float,
        default=30,
        help="Maximum frames per second for rendering",
    )
    parser.add_argument(
        "--marker-size", type=float, default=8, help="Size of markers in plots"
    )
    parser.add_argument(
        "--show-legend", action="store_true", help="Show drone IDs legend"
    )
    return parser.parse_args()


def configure_axis(ax, view, bound):
    labels = {
        "top": ("X", "Y", "Top View (XY)"),
        "front": ("Y", "Z", "Front View (YZ)"),
        "side": ("X", "Z", "Side View (XZ)"),
        "3d": ("X", "Y", "Z", "3D View"),
    }

    if view == "3d":
        ax.set_title(labels[view][3])
        ax.set_xlim3d(-bound, bound)
        ax.set_ylim3d(-bound, bound)
        ax.set_zlim3d(-bound, bound)
        ax.set_xlabel(labels[view][0])
        ax.set_ylabel(labels[view][1])
        ax.set_zlabel(labels[view][2])
        ax.view_init(elev=30, azim=-60)
    else:
        ax.set_title(labels[view][2])
        ax.set_xlabel(labels[view][0])
        ax.set_ylabel(labels[view][1])
        ax.set_xlim(-bound, bound)
        ax.set_ylim(-bound, bound)
        ax.set_aspect("equal")
    ax.grid(True)


class AnimationState:
    def __init__(self):
        self.last_render_time = 0
        self.current_idx = 0


def main():
    args = parse_args()

    # Настройка бэкенда
    matplotlib.use("WebAgg" if args.backend == "web" else "QtAgg")

    # Загрузка и предобработка данных
    data = np.load(args.file)
    time = data["time"]
    states = data["states"]

    # Применяем прореживание кадров
    if args.decimate > 1:
        time = time[:: args.decimate]
        states = states[:: args.decimate]

    T, N, _ = states.shape
    # frame_interval = 1000 / args.max_fps  # ms per frame
    frame_interval = (args.dt * args.decimate * 1000) / args.speed

    # Рассчет временных параметров
    speed_factor = args.speed  #  * (adjusted_dt / original_dt)

    # Создание фигуры
    fig = plt.figure(figsize=(args.width, args.height))
    ax_top = fig.add_subplot(221)
    ax_front = fig.add_subplot(222)
    ax_side = fig.add_subplot(223)
    ax_3d = fig.add_subplot(224, projection="3d")

    # Инициализация графиков
    marker_size = args.marker_size
    initial_pos = states[0]
    scat_top = ax_top.scatter(
        initial_pos[:, 0], initial_pos[:, 1], s=marker_size**2, c="blue"
    )
    scat_front = ax_front.scatter(
        initial_pos[:, 1], initial_pos[:, 2], s=marker_size**2, c="green"
    )
    scat_side = ax_side.scatter(
        initial_pos[:, 0], initial_pos[:, 2], s=marker_size**2, c="red"
    )
    scat_3d = ax_3d.scatter(
        initial_pos[:, 0],  # X координаты
        initial_pos[:, 1],  # Y координаты
        initial_pos[:, 2],  # Z координаты
        s=marker_size**2,
        c="purple",
    )

    # Настройка осей
    for ax, view in zip(
        [ax_top, ax_front, ax_side, ax_3d], ["top", "front", "side", "3d"]
    ):
        configure_axis(ax, view, args.bound)

    # Информационная панель
    info_text = fig.text(
        0.5,
        0.93,
        "",
        ha="center",
        fontsize=9,
        bbox=dict(facecolor="white", alpha=0.8),
    )

    # Легенда
    if args.show_legend and N <= 50:
        handles = [
            Line2D(
                [],
                [],
                marker="o",
                color="w",
                markerfacecolor="b",
                markersize=6,
                label=str(i),
            )
            for i in range(N)
        ]
        fig.legend(
            handles=handles,
            title="Drone IDs",
            loc="center right",
            ncol=1,
            fontsize="xx-small",
            bbox_to_anchor=(0.98, 0.5),
        )

    # Состояние анимации
    anim_state = AnimationState()

    # Функция обновления
    def update(frame):
        current_idx = anim_state.current_idx
        if current_idx >= T:
            anim_state.current_idx = 0
            current_idx = 0

        state = states[current_idx]
        scat_top.set_offsets(state[:, :2])
        scat_front.set_offsets(state[:, 1:])
        scat_side.set_offsets(state[:, [0, 2]])
        scat_3d._offsets3d = (state[:, 0], state[:, 1], state[:, 2])

        # Обновление информации
        info_text.set_text(
            f"Time: {time[current_idx]:.1f}s / {time[-1]:.1f}s | "
            f"Speed: {speed_factor:.1f}x | "
            f"Frames: {current_idx + 1}/{T} | "
            f"Decimate: {args.decimate} | "
            f"Rendering FPS: {args.max_fps}"
        )

        anim_state.current_idx += 1
        return [scat_top, scat_front, scat_side, scat_3d, info_text]

    # Создание анимации
    ani = FuncAnimation(  # noqa: F841
        fig,
        update,
        interval=frame_interval,
        blit=False,
        save_count=T,
        cache_frame_data=False,
    )

    plt.tight_layout(rect=[0, 0, 0.98, 0.95])
    plt.show()


if __name__ == "__main__":
    main()
