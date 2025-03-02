import argparse
import time
from pion import Spion
from pion.server import SwarmCommunicator  # модуль, где определён SwarmCommunicator
from pion.control_server import get_local_ip

def main():
    parser = argparse.ArgumentParser(
        description="Запуск SwarmCommunicator для управления дроном"
    )
    parser.add_argument(
        "device_number",
        type=str,
        help="Номер экземпляра для формирования уникального id, например, '1' для первого, '2' для второго и т.д."
    )
    args = parser.parse_args()
    ip = get_local_ip()

    # Создаём объект дрона (Spion или Pion)
    drone = Spion(ip=ip, mavlink_port=5656, name="Drone", dt=0.01, logger=True, max_speed=0.5)

    # Передаём device_number в SwarmCommunicator для генерации уникального id
    swarm_comm = SwarmCommunicator(control_object=drone, broadcast_port=37020, broadcast_interval=0.5, ip=ip, instance_number=args.device_number)
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {drone.name} с IP {ip} и уникальным id {swarm_comm.unique_id}")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        swarm_comm.stop()
        print("Swarm communicator остановлен.")

if __name__ == "__main__":
    main()
