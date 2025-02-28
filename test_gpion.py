import argparse
import time
from pion.gpion import Gpion
from pion.server import SwarmCommunicator

def main():
    parser = argparse.ArgumentParser(
        description="Запуск SwarmCommunicator для управления дроном на базе Gpion"
    )
    parser.add_argument(
        "device_number",
        type=str,
        help="Номер устройства для формирования IP, например, '3' для IP 10.1.100.3"
    )
    parser.add_argument(
        "--ssh_host",
        type=str,
        default=None,
        help="SSH хост для проверки Pion server (например, 10.1.100.121)"
    )
    parser.add_argument(
        "--ssh_user",
        type=str,
        default="pi",
        help="SSH пользователь (по умолчанию 'pi')"
    )
    parser.add_argument(
        "--ssh_password",
        type=str,
        default="raspberry",
        help="SSH пароль (по умолчанию 'raspberry')"
    )
    args = parser.parse_args()

    # Формируем IP по шаблону "10.1.100.{number}"
    ip = f"10.1.100.{args.device_number}"
    
    # Создаем экземпляр Gpion
    drone = Gpion(ip=ip, mavlink_port=5656, name=f"Drone_{args.device_number}", dt=0.1)
    
    # Если указан SSH-хост, проверяем запущенность Pion server
    if args.ssh_host:
        drone.check_pion_server(args.ssh_host, args.ssh_user, args.ssh_password)
    
    # Создаем SwarmCommunicator для обмена состоянием/командами
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

