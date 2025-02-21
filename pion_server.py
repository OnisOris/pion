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

    # Формирование IP по шаблону "10.1.100.{number}"
    ip = f"10.1.100.{args.device_number}"
    
    # Создаем экземпляр дрона (например, Pion) с заданным IP.
    # Здесь mavlink_port и dt заданы как пример, их можно настроить под свои нужды.
    drone = Spion(ip=ip, mavlink_port=5656, name=f"Drone_{args.device_number}", dt=0.1)
    
    # Создаем SwarmCommunicator, передавая объект управления под именем control_object
    # (внутри класса SwarmCommunicator вместо self.pion используется self.control_object)
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
