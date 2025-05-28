#!/usr/bin/env python3
import argparse
import os
import time

import numpy as np
from dotenv import load_dotenv

from pion import Pion
from pionfunc.functions import get_local_ip
from swarm_server import SwarmCommunicator

os.environ["GRPC_DNS_RESOLVER"] = "native"

load_dotenv()
parser = argparse.ArgumentParser(description="Запуск дрона для записи данных")
parser.add_argument(
    "--log",
    type=bool,
    default=False,
    help="Rich logs",
)
args = parser.parse_args()

KPx = float(os.getenv("KPx"))
KPy = float(os.getenv("KPy"))
KPz = float(os.getenv("KPz"))
KIx = float(os.getenv("KIx"))
KIy = float(os.getenv("KIy"))
KIz = float(os.getenv("KIz"))
KDx = float(os.getenv("KDx"))
KDy = float(os.getenv("KDy"))
KDz = float(os.getenv("KDz"))
attraction_weight = float(os.getenv("attraction_weight"))
cohesion_weight = float(os.getenv("cohesion_weight"))
alignment_weight = float(os.getenv("alignment_weight"))
repulsion_weight = float(os.getenv("repulsion_weight"))
unstable_weight = float(os.getenv("unstable_weight"))
current_velocity_weight = float(os.getenv("current_velocity_weight"))
noise_weight = float(os.getenv("noise_weight"))
safety_radius = float(os.getenv("safety_radius"))
max_acceleration = float(os.getenv("max_acceleration"))
max_speed = float(os.getenv("max_speed"))
unstable_radius = float(os.getenv("unstable_radius"))


params = {
    "kp": np.array([[KPx, KPy, KPz, 0, 0, 0]]),
    "ki": np.array([[KIx, KIy, KIz, 0, 0, 0]]),
    "kd": np.array([[KDx, KDy, KDz, 0, 0, 0]]),
    "attraction_weight": attraction_weight,
    "cohesion_weight": cohesion_weight,
    "alignment_weight": alignment_weight,
    "repulsion_weight": repulsion_weight,
    "unstable_weight": unstable_weight,
    "current_velocity_weight": current_velocity_weight,
    "noise_weight": noise_weight,
    "safety_radius": safety_radius,
    "max_acceleration": max_acceleration,
    "max_speed": max_speed,
    "unstable_radius": unstable_radius,
}


def main():
    ip = get_local_ip()
    drone = Pion(
        ip="localhost",
        mavlink_port=5656,
        connection_method="udpout",
        name=f"Drone-{ip}",
        dt=0.001,
        logger=args.log,
        max_speed=0.5,
    )

    swarm_comm = SwarmCommunicator(
        control_object=drone,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=ip,
        time_sleep_update_velocity=0.1,
        params=params,
    )
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {drone.name} с IP {ip}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        swarm_comm.stop()
        print("Swarm communicator остановлен.")


if __name__ == "__main__":
    main()
