import time
import threading
from pion import Spion  # Предполагается, что Spion уже реализован в модуле pion
from pion.server import SwarmCommunicator

def start_drone(name: str, ip: str) -> None:
    """
    Функция создает экземпляр дрона (Spion) с заданным именем и IP,
    запускает для него SwarmCommunicator и держит его в рабочем цикле.
    
    :param name: Имя дрона.
    :param ip: IP-адрес дрона.
    """
    # Создаем дрона. Обратите внимание, что Spion создается с параметрами:
    # ip, mavlink_port, mass, dt, logger и name.
    drone = Spion(ip=ip, mavlink_port=5656, mass=0.3, dt=0.05, logger=True, name=name)
    drone.arm()
    drone.takeoff()
    drone.goto(1, 0, 3, 1)
    # Инициализируем коммуникатор для роевого обмена.
    communicator = SwarmCommunicator(pion=drone, broadcast_port=37020, broadcast_interval=0.5)
    communicator.start()
    print(f"{name} started with IP {ip}.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        communicator.stop()
        print(f"Swarm communicator for {name} stopped.")

def main() -> None:
    # Задаем информацию для 4 дронов: имя и уникальный IP-адрес.
    drones_info = [
        ("Drone_1", "10.1.100.201"),
        ("Drone_2", "10.1.100.202"),
        ("Drone_3", "10.1.100.203"),
        ("Drone_4", "10.1.100.204")
    ]
    
    threads = []
    for name, ip in drones_info:
        t = threading.Thread(target=start_drone, args=(name, ip), daemon=True)
        t.start()
        threads.append(t)
    
    print("All drones are running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting swarm example.")

if __name__ == "__main__":
    main()
