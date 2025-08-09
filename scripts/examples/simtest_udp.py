import sys
from pion.spion import Spion
from pionfunc.functions import start_threading
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
    "current_velocity_weight": 1.0,
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
    attitude = np.array([0, 0, np.pi/3, 0, 0, 0])
    # Получаем локальный IP-адрес
    sim_drone = Spion(
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
        control_object=sim_drone,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=str(args.id),
        time_sleep_update_velocity=0.1,
        params=params,
        unique_id=args.id,
    )
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {sim_drone.name}")
    return swarm_comm

# drone = drone_init()
# print("---")
# print("Main arm ------------<")
# drone.control_object.arm()
# print("takeoff -----------------<")
# drone.control_object.takeoff()
# time.sleep(5)
# print("goto --------------<")
# drone.control_object.goto_from_outside(5, 1, 1, 1)
#
# body_target = np.array([1, 0, 0])
# drone.control_object.goto_body(body_target)
# print("land ----------------<")
# drone.control_object.land()
# print("stop ----------------<")
# drone.stop()


drone = drone_init()
print("---")
print("Main arm ------------<")
drone.control_object.arm()
print("takeoff -----------------<")
# drone.control_object.takeoff()
# time.sleep(5)
# print("goto --------------<")
# drone.co.set_body_velocity(np.array([1, 0, 0, 0]))
drone.control_object.goto(2.6, -2.7, 0, 0)