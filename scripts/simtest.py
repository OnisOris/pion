import sys
import time

from pion.spion import Spion

# Демонстрационный полет симуляционного дрона
args = sys.argv
drone = Spion(
    ip="10.1.100.200", mavlink_port=5656, mass=0.3, dt=0.05, logger=True
)
print("---")
print("Main arm ------------<")
drone.arm()
print("takeoff -----------------<")
drone.takeoff()
time.sleep(5)
print("goto --------------<")
drone.goto_from_outside(5, 1, 1, 1)
print("land ----------------<")
drone.land()
print("stop ----------------<")
drone.stop()
