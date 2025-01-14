from pion import Spion
import sys
import time

args = sys.argv

drone = Spion(ip=f"10.1.100.200", mavlink_port=5656, mass=0.3, dt=0.05, logger=True, dimension=2)
print("---")
print("Main arm ------------<")
drone.arm()
print("takeoff -----------------<")
drone.takeoff()
time.sleep(5)

print("goto --------------<")

drone.goto(20, 1, 1, 0)
print("land ----------------<")

drone.land()

print("stop ----------------<")

drone.stop()
