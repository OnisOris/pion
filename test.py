from pion import Pion
import sys
import time

args = sys.argv

number_drone = sys.argv[1]
drone = Pion(ip=f"10.1.100.{number_drone}", mavlink_port=5656, logger=True)
drone.speed_flag = False
if '-c' in args:
    while True:
        print(drone.attitude)
        time.sleep(0)
else:
    print("---")
    drone.land()
    drone.arm()
    drone.takeoff()
    time.sleep(5)
    drone.set_v()
    drone.goto_from_outside(float(args[2]), float(args[3]), float(args[4]), float(args[5]))
    drone.stop()
    drone.land()

