import argparse
import time

from pion import Pion

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Запуск слежения за траекторией"
    )
    parser.add_argument(
        "--file",
        type=str,
        default="data.npy",
        help="Path to .npy file with arrays 'time' and 'positions'",
    )
    parser.add_argument(
        "--num",
        type=int,
        default=153,
        help="Number of drone",
    )
    args = parser.parse_args()

    drone = Pion(
        ip=f"10.1.100.{args.num}",
        mavlink_port=5656,
        logger=True,
        dt=0.0,
        count_of_checking_points=5,
    )

    drone.arm()
    drone.takeoff()
    time.sleep(5)
    drone.trajectory_tracking(args.file)
    drone.stop()
