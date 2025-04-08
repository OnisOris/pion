#!/usr/bin/env python3
import argparse
import time

from pion.spion import Spion
from pionfunc.functions import get_local_ip
from swarm_server import SwarmCommunicator

from .params import params


def main():
    parser = argparse.ArgumentParser(description="Симуляция члена роя")
    parser.add_argument(
        "--id",
        default=100,
        help="id дрона",
    )
    args = parser.parse_args()
    # Получаем локальный IP-адрес
    ip = get_local_ip()
    drone = Spion(
        ip="/dev/ttyS0",
        mavlink_port=230400,
        connection_method="serial",
        name=f"Drone-{ip}-{args.id}",
        dt=0.001,
        logger=False,
        max_speed=0.5,
    )

    swarm_comm = SwarmCommunicator(
        control_object=drone,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=str(args.id),
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
