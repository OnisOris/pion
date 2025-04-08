#!/usr/bin/env python3

import time

from pion import Pion
from pionfunc.functions import get_local_ip
from swarm_server import SwarmCommunicator

from .params import params


def main():
    # Получаем локальный IP-адрес
    ip = get_local_ip()
    drone = Pion(
        ip="/dev/ttyS0",
        mavlink_port=230400,
        connection_method="serial",
        name=f"Drone-{ip}",
        dt=0.001,
        logger=True,
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
