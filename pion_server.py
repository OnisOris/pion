import argparse
import time
from pion import Spion
from pion.server import SwarmCommunicator  # модуль, где определён SwarmCommunicator

def main():
    parser = argparse.ArgumentParser(
        description="Запуск SwarmCommunicator для управления дроном"
    )
    parser.add_argument(
        "device_number",
        type=str,
        help="Номер устройства для формирования IP, например, '3' для IP 10.1.100.3"
    )
    args = parser.parse_args()
    ip = f"10.1.100.{args.device_number}"

    drone = Spion(ip=ip, mavlink_port=5656, name=f"Drone_{args.device_number}", dt=0.01, logger=True, max_speed=0.5)

    swarm_comm = SwarmCommunicator(control_object=drone, broadcast_port=37020, broadcast_interval=0.5)
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
