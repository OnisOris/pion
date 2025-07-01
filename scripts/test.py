import time
from argparse import ArgumentParser

import numpy as np

from pion import Pion

parser = ArgumentParser(description="Набор тестовых программ для отладки.")
parser.add_argument(
    "drone_number",
    type=int,
    help="Number of drone",
)

group = parser.add_mutually_exclusive_group()
group.add_argument(
    "-c",
    "--coordinate",
    action="store_true",
    help="Show coordinates for debugging",
)
group.add_argument(
    "-l",
    "--land",
    action="store_true",
    help="Land drone",
)
group.add_argument(
    "-d",
    "--disarm",
    action="store_true",
    help="Disarm drone",
)
group.add_argument(
    "-at",
    "--arm_takeoff",
    action="store_true",
    help="Arm and takeoff drone",
)
group.add_argument(
    "-tr",
    "--track_point",
    nargs=4,
    metavar=("x", "y", "z", "yaw"),
    type=float,
    help="Start traking point",
)
group.add_argument(
    "-r",
    "--reboot",
    action="store_true",
    help="Reboot autopilot of drone",
)
group.add_argument(
    "--yaw",
    type=float,
    help="Set yaw value for drone",
)
group.add_argument(
    "-gt",
    "--goto",
    nargs=4,
    metavar=("x", "y", "z", "yaw"),
    type=float,
    help="Move drone to position",
)


def main(drone: Pion, args):
    np.set_printoptions(suppress=True)

    if args.coordinate:
        drone.led_control(255, 0, 255, 0)
        drone.logger = True
        while True:
            time.sleep(0.02)

    if args.land:
        drone.led_control(255, 0, 0, 0)
        drone.land()

    if args.disarm:
        drone.led_control(255, 255, 0, 0)
        drone.disarm()
        drone.led_control(255, 0, 0, 0)

    if args.arm_takeoff:
        print("Run arm takeoff")
        drone.led_control(255, 255, 0, 0)
        drone.arm()
        drone.takeoff()

    if args.reboot:
        drone.reboot_board()

    if args.track_point:
        drone.arm()
        drone.takeoff()
        time.sleep(8)
        drone.target_point = np.array(args.track_point)
        drone.start_track_point()
        while True:
            time.sleep(1)

    if args.yaw:
        drone.goto_yaw(args.yaw)

    if args.goto:
        print(*args.goto)
        drone.arm()
        drone.takeoff()
        time.sleep(8)
        drone.set_v()
        drone.goto(
            *args.goto,
            autopilot_controller=False,
            wait=True,
        )
        drone.stop()


if __name__ == "__main__":
    args = parser.parse_args()

    drone = Pion(
        ip=f"10.1.100.{args.drone_number}",
        mavlink_port=5656,
        logger=True,
        dt=0.0,
        count_of_checking_points=5,
    )
    try:
        main(drone, args)
    except KeyboardInterrupt:
        print("Work finished.")
    finally:
        drone.stop()
