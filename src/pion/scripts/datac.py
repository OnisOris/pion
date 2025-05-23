import argparse
import datetime
import time
from os.path import isdir

import numpy as np

from pion import Pion, Spion

"""
Скрипт для аналитики работы дрона
"""


def main():
    parser = argparse.ArgumentParser(
        description="Запуск дрона для записи данных"
    )
    parser.add_argument(
        "--sim", action="store_true", help="Запуск в симуляции"
    )
    parser.add_argument("--led", action="store_true", help="Led indication")
    parser.add_argument(
        "--path",
        type=str,
        default="./",
        help="onle path to folder",
    )

    parser.add_argument(
        "--ip",
        type=str,
        default="10.1.100.133",
        help="ip address",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=5656,
        help="mavlink port",
    )

    parser.add_argument(
        "--kind",
        type=str,
        default="step_x",
        help="Kind of experiment",
    )
    parser.add_argument(
        "--logger",
        type=bool,
        default=True,
        help="Kind of experiment",
    )
    parser.add_argument(
        "--d",
        type=str,
        default="",
        help="Description of experiment",
    )

    args = parser.parse_args()
    if args.sim:
        drone = Spion(logger=True)
    else:
        drone = Pion(
            ip=args.ip,
            mavlink_port=args.port,
            logger=args.logger,
        )
    drone.check_attitude_flag = True
    match args.kind:
        case "step_x":
            drone.arm()
            if args.led:
                drone.led_control(255, 0, 255, 0)
            drone.stakeoff(1)
            drone.goto_yaw(0.0)
            drone.goto_from_outside(-4, 0, 1.5, 0, wait=True)
            drone.set_v()
            drone.t_speed = np.array([1, 0, 0, 0])
            time.sleep(4)
            drone.t_speed = np.array([0, 0, 0, 0])
            drone.stop()
            drone.land()
        case "step_y":
            drone.arm()
            if args.led:
                drone.led_control(255, 0, 255, 0)
            drone.stakeoff(1)
            drone.goto_yaw(0.0)
            drone.goto_from_outside(0, 4, 1.5, 0, wait=True)
            drone.set_v()
            drone.t_speed = np.array([0, -1, 0, 0])
            time.sleep(4)
            drone.t_speed = np.array([0, 0, 0, 0])
            drone.stop()
            drone.land()
    current_date = datetime.date.today().isoformat()
    current_time = str(datetime.datetime.now().time())
    symbols_to_remove = ":"

    if not isdir(args.path):
        from os import makedirs

        makedirs(args.path, exist_ok=True)

    description = args.d

    for symbol in symbols_to_remove:
        current_time = current_time.replace(symbol, "-")

    description = description.replace(" ", "")

    drone.save_data(
        f"{args.path}data_description_{description}_{drone.ip}_{current_date}_{current_time}.npy"
    )


if __name__ == "__main__":
    main()
