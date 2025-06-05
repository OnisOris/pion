import sys
import time

import numpy as np

from pion import Pion

np.set_printoptions(suppress=True)  # Отключить экспоненциальный формат
args = sys.argv
number_drone = sys.argv[1]
drone = Pion(
    ip=f"10.1.100.{number_drone}",
    mavlink_port=5656,
    logger=True,
    dt=0.0,
    count_of_checking_points=5,
)
if "-c" in args:
    drone.led_control(255, 0, 255, 0)
    drone.logger = True
    while True:
        time.sleep(0.02)
elif "-l" in args:
    drone.led_control(255, 0, 0, 0)
    drone.land()

elif "-d" in args:
    drone.led_control(255, 255, 0, 0)
    drone.disarm()
    drone.led_control(255, 0, 0, 0)
elif "-at" in args:
    drone.led_control(255, 255, 0, 0)
    drone.arm()
    drone.takeoff()
elif "-r" in args:
    drone.reboot_board()
elif "-tr" in args:
    drone.arm()
    drone.takeoff()
    time.sleep(8)
    drone.start_track_point()
    while True:
        time.sleep(1)
elif "-yaw" in args:
    drone.goto_yaw(float(sys.argv[3]))
else:
    print("---")
    drone.arm()
    drone.takeoff()
    time.sleep(8)
    drone.set_v()
    drone.goto_from_outside(
        float(args[2]),
        float(args[3]),
        float(args[4]),
        float(args[5]),
        wait=True,
    )
    drone.stop()
