from pion import Pion
import time

if __name__ == "__main__":
    drone = Pion(
        ip=f"10.1.100.134",
        mavlink_port=5656,
        logger=True,
        dt=0.0,
        count_of_checking_points=5,
    )
    drone.arm()
    drone.takeoff()
    time.sleep(15)
    drone.land()
    drone.stop()