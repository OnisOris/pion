#!/usr/bin/env python3
import time
from typing import Any, Optional

import numpy as np

from pion import Pion
from pion.cython_pid import PIDController
from pionfunc.functions import compute_swarm_velocity_pid, get_local_ip
from swarm_server import SwarmCommunicator

from .params import params


class Swarmc(SwarmCommunicator):
    def __init__(
        self,
        control_object: Any,
        broadcast_port: int = 37020,
        broadcast_interval: float = 0.05,
        safety_radius: float = 1.0,
        max_speed: float = 1.0,
        ip=None,
        instance_number=None,
        time_sleep_update_velocity: float = 0.1,
        params: Optional[dict] = None,
        d: int = 2,
    ):
        SwarmCommunicator.__init__(
            self,
            control_object=control_object,
            broadcast_port=broadcast_port,
            broadcast_interval=broadcast_interval,
            safety_radius=safety_radius,
            max_speed=max_speed,
            ip=ip,
            instance_number=instance_number,
            time_sleep_update_velocity=time_sleep_update_velocity,
            params=params,
        )
        self.position_pid_matrix = np.array(
            [[0.05] * d, [0.0] * d, [0.7] * d], dtype=np.float64
        )
        self._pid_position_controller: Optional[PIDController] = None

    def update_swarm_control(self, target_point, dt) -> None:
        signal = np.clip(
            self._pid_position_controller.compute_control(
                target_position=np.array(target_point, dtype=np.float64),
                current_position=self.control_object.xyz[0:2],
                dt=dt,
            ),
            -self.max_speed,
            self.max_speed,
        )
        swarm_part = compute_swarm_velocity_pid(
            self.control_object.position,
            self.env,
            target_point,
            params=self.params,
        )
        new_vel = signal + swarm_part
        self.control_object.t_speed = np.array([new_vel[0], new_vel[1], 0, 0])

    def smart_point_tacking(self):
        print("Smart point tracking")
        self.control_object.set_v()
        self.control_object.point_reached = False
        self.control_object.tracking = True
        self._pid_position_controller = PIDController(
            *self.position_pid_matrix
        )
        last_time = time.time()
        time.sleep(self.time_sleep_update_velocity)
        while self.control_object.tracking:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            self.update_swarm_control(
                self.control_object.target_point[0:2], dt
            )
            time.sleep(self.time_sleep_update_velocity)
        self.t_speed = np.zeros(4)


def main():
    # Получаем локальный IP-адрес
    ip = get_local_ip()
    drone = Pion(
        ip="localhost",
        mavlink_port=5656,
        connection_method="udpout",
        name=f"Drone-{ip}",
        dt=0.001,
        logger=True,
        max_speed=0.5,
    )

    swarm_comm = Swarmc(
        control_object=drone,
        broadcast_port=37020,
        broadcast_interval=0.5,
        ip=ip,
        time_sleep_update_velocity=0.1,
        params=params,
    )
    swarm_comm.start()
    print(f"SwarmCommunicator запущен для {drone.name} с IP {ip}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        swarm_comm.stop()
        print("Swarm communicator остановлен.")


if __name__ == "__main__":
    main()
