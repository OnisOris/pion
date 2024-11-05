from pion import Spion
import sys
import time

args = sys.argv

drone = Spion(ip=f"10.1.100.200", mavlink_port=5656, mass=0.3, dt=0.001)

drone.max_speed = 1000
print("---")
drone.set_v()

print("Main arm ------------<")

drone.arm()

print("takeoff -----------------<")

drone.takeoff()

print("goto --------------<")

drone.goto(20, 1, 1, 0)
print("land ----------------<")

drone.land()

print("stop ----------------<")

drone.stop()
