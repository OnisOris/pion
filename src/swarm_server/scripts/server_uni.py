#!/usr/bin/env python3
import time
import argparse

from pion import Pion
from pionfunc.functions import get_local_ip
from swarm_server import SwarmCommunicator


from .params import params

def main():
    parser = argparse.ArgumentParser(
        description="Запуск универсального сервера для управления единицей роя"
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="localhost",
        help="ip устройства (например, 10.1.100.121)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Порт устройства, например 5656",
    )
    args = parser.parse_args()
    ip = get_local_ip()
    drone = Pion(
        ip=args.ip,
        mavlink_port=args.port,
        connection_method="udpout",
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
