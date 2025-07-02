from pion import Pion
from pionfunc.functions import create_connection

if __name__ == "__main__":
    drone = Pion(
        ip=f"10.1.100.1",
        mavlink_port=5656,
        logger=True,
        dt=0.0,
        count_of_checking_points=5,
    )
    for num in range(100, 200):
        drone.mavlink_socket = create_connection(
            connection_method=drone.connection_method,
            address=f"10.1.100.{num}",
            port_or_baudrate=drone.mavlink_port,
        )
        drone.reboot_board()
        drone.stop()
