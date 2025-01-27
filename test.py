from pion import Pion
import sys
import time

args = sys.argv

number_drone = sys.argv[1]
drone = Pion(ip=f"10.1.100.{number_drone}", mavlink_port=5656, logger=True)
if '-c' in args:
    drone.led_control(255, 0, 255, 0)
    while True:
        print(drone.attitude)
        time.sleep(0)
        time.sleep(0.2)
if '-l' in args:
    drone.led_control(255, 0, 0, 0)
    drone.land()
if '-r' in args:
    drone.reboot_board()

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

