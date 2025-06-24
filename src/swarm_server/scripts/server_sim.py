#!/usr/bin/env python3
import argparse
import time

import numpy as np

from pion.spion import Spion
from swarm_server import SwarmCommunicator


params = {
    "kp": np.array([[1, 1, 1, 1, 1, 1]]) * 0.2,
    "ki": np.zeros((1, 6)),
    "kd": np.array([[1, 1, 1, 1, 1, 1]]),
    "attraction_weight": 1.0,
    "cohesion_weight": 1.0,
    "alignment_weight": 1.0,
    "repulsion_weight": 4.0,
    "unstable_weight": 1.0,
    "noise_weight": 1.0,
    "safety_radius": 1,
    "max_acceleration": 10,
    "max_speed": 1,
    "unstable_radius": 1.5,
}


def drone_init():
    parser = argparse.ArgumentParser(description="Симуляция члена роя")
    parser.add_argument(
        "--id",
        default=100,
        help="id дрона",
        type=int,
    )
    parser.add_argument(
        "--initial",
        type=float,
        nargs=4,  # ожидаем 4 значения: x, y, z, yaw
        metavar=("x", "y", "z", "yaw"),
        default=[0.0, 0.0, 0.0, 0.0],
        help="Начальные координаты (x, y, z) и угол поворота (yaw)",
    )
    args = parser.parse_args()

    # Разбираем входные координаты
    x, y, z, yaw = args.initial
    position = np.array([x, y, z, 0, 0, 0])
    attitude = np.array([0, 0, yaw, 0, 0, 0])
    args = parser.parse_args()
    # Получаем локальный IP-адрес
    drone = Spion(
        ip="/dev/ttyS0",
        mavlink_port=230400,
        connection_method="serial",
        name=f"Drone-localhost-{args.id}",
        dt=0.1,
        logger=False,
        max_speed=0.5,
        position=position,
        attitude=attitude,
    )

    swarm_comm = SwarmCommunicator(
        control_object=drone,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=str(args.id),
        time_sleep_update_velocity=0.1,
        params=params,
        unique_id=args.id,
    )
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {drone.name}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        swarm_comm.stop()
        print("Swarm communicator остановлен.")
    print("Drone_init")
    return swarm_comm


if __name__ == "__main__":
    drone_init()
