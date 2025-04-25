import threading
import time
from collections import deque
from typing import List, Dict, Optional, Any

import numpy as np
from lokky.pionmath import SSolver
from numpy.typing import NDArray
from rich.live import Live
from rich.table import Table
import json


from pion.cython_pid import (
    PIDControllerXd,
)
from pion.simulator import PointYaw, Simulator_realtime_th


def create_objects_Point_yaw(
    number: int = 10,
    linspace_x: list = [-5, 5],
    linspace_y: list = [-5, 5],
    z_offset: float = 0.,
) -> NDArray:
    x1 = np.linspace(*linspace_x, num=number)
    x2 = np.linspace(*linspace_y, num=number)
    x, y = np.meshgrid(x1, x2)
    coords = np.vstack([x.ravel(), y.ravel()]).T
    coords6 = np.c_[coords, np.zeros((coords.shape[0], 4))]
    coords6[0, 2] += z_offset
    points = np.array([])
    for coord in coords6:
        points = np.hstack([points, PointYaw(position=coord)])
    return points


class SwarmSim(Simulator_realtime_th):
    """ """

    def __init__(
        self,
        simulation_objects: np.ndarray,
        dt: float = 0.01,
        dimension: int = 3,
        max_speed: float = 1,
        max_acceleration: float = 10,
        logger: bool = False,
        params: Optional[dict] = None,
    ):
        super().__init__(simulation_objects, dt, dimension)
        # Количество объектов (роя)
        self.n_objects = simulation_objects.shape[0]
        # Количество каналов для контроля (например, скорость по dimension + yaw rate)
        self.channels = self.dimension + 1

        # Создание матриц коэффициентов для PID:
        # Для каждого объекта (n_objects) задаем коэффициенты для каждого канала (channels)
        self.kp: NDArray[Any] = np.full(
            (self.n_objects, self.channels), 3.0, dtype=np.float64
        )
        self.ki: NDArray[Any] = np.full(
            (self.n_objects, self.channels), 0.0, dtype=np.float64
        )
        self.kd: NDArray[Any] = np.full(
            (self.n_objects, self.channels), 0.1, dtype=np.float64
        )
        self.t_speed = np.zeros((self.n_objects, 4))
        self.t_position = np.zeros((self.n_objects, 6))

        self._pid_velocity_controller: PIDControllerXd = PIDControllerXd(
            self.kp, self.ki, self.kd
        )
        self.max_speed: float = max_speed
        self.max_acceleration: float = max_acceleration
        self.logs: dict = {}
        self.logger: bool = logger
        self._live: Optional[Live] = None
        if params is None:
            self.params: dict = {
                "kp": np.ones((self.n_objects, 6)),
                "ki": np.zeros((self.n_objects, 6)),
                "kd": np.ones((self.n_objects, 6)) * 0.1,
                "attraction_weight": 1.0,
                "cohesion_weight": 1.0,
                "alignment_weight": 1.0,
                "repulsion_weight": 80.0,
                "unstable_weight": 1.0,
                "noise_weight": 1.0,
                "safety_radius": 1,
                "max_acceleration": 2,
                "max_speed": 0.4,
                "unstable_radius": 1.5,
            }
        else:
            self.params: dict = params
        self.swarmsolver = SSolver(
            params=self.params, count_of_objects=simulation_objects.shape[0]
        )
        self.swarm_on: bool = True

    def set_velocity_pid(self) -> None:
        """
        Инициализирует PID контроллер для скорости с матрицами коэффициентов,

        размерность которых соответствует количеству объектов и каналов.
        """
        self._pid_velocity_controller = PIDControllerXd(
            self.kp, self.ki, self.kd
        )

    def velocity_controller(self) -> None:
        """
        Метод вычисления управляющего воздействия для всех объектов симуляции.

        Предполагается, что:
          - self.t_speed содержит желаемые значения скоростей, имеющие форму (n_objects, channels)
          - Каждый объект имеет:
                • атрибут speed: массив длины self.dimension,
                • атрибут attitude: массив, где элемент с индексом 5 – это текущая угловая скорость (yaw rate).
        """
        # Приводим целевые скорости к нужному формату (двумерный массив: (n_objects, channels))
        target_speed = np.array(self.t_speed, dtype=np.float64)

        # Формирование массива текущих скоростей для всех объектов
        current_velocities = np.empty(
            (self.n_objects, self.channels), dtype=np.float64
        )
        for i, sim_obj in enumerate(self.simulation_objects):
            current_yaw_rate = sim_obj.attitude[5]
            current_velocities[i, :] = np.hstack(
                [sim_obj.speed, current_yaw_rate]
            )

        # Вычисление управляющего сигнала посредством PID
        signal = self._pid_velocity_controller.compute_control(
            target_position=target_speed,
            current_position=current_velocities,
            dt=self.dt,
        )

        # Ограничение сигнала по максимальному ускорению
        signal = np.clip(signal, -self.max_acceleration, self.max_acceleration)
        # Применяем рассчитанную силу к моделям (например, для всех объектов сразу)
        self.forces = signal

    def object_cycle(
        self,
        simulation_object,
        object_channel: int,
        type_of_cycle: str = "while",
        steps: int = 100,
    ) -> None:
        """
        Метод, выполняемый в отдельном потоке для каждого объекта.

        После выполнения шага симуляции поток ждёт, пока все остальные потоки не вызовут sync_barrier.wait().
        Таким образом синхронизуются шаги.
        """
        last_time = time.time()
        if type_of_cycle == "while":
            while self.simulation_turn_on:
                current_time = time.time()
                elapsed_time = current_time - last_time
                if elapsed_time >= self.dt:
                    # print(f"obj_channel = ", object_channel, "pos = ", self.simulation_objects[object_channel].state)
                    last_time = current_time
                    if self.swarm_on:
                        self.swarm_controller(self.dt)
                    self.velocity_controller()
                    if self.logger:
                        self.print_information()
                    simulation_object.step(
                        self.forces[object_channel], self.dt
                    )
                    try:
                        self.sync_barrier.wait()
                    except threading.BrokenBarrierError:
                        break
        elif type_of_cycle == "for":
            for _ in range(steps):
                if not self.simulation_turn_on:
                    break
                self.velocity_controller()
                if self.logger:
                    self.print_information()
                simulation_object.step(self.forces[object_channel], self.dt)
                try:
                    self.sync_barrier.wait()
                except threading.BrokenBarrierError as e:
                    print(e)
                    break
        else:
            print("Неизвестный тип цикла! Выберите 'while' или 'for'.")

    def print_information(self) -> None:
        """
        Выводит в реальном времени таблицу состояний всех объектов.

        Колонки: ID, x, y, z, vx, vy, vz, yaw, t_speed_x, t_speed_y, t_speed_z, t_yaw.
        """
        # Создаем таблицу
        table = Table(title="Swarm Simulation Status")
        table.add_column("ID", style="cyan", justify="right")
        table.add_column("x", style="green")
        table.add_column("y", style="green")
        table.add_column("z", style="green")
        table.add_column("vx", style="yellow")
        table.add_column("vy", style="yellow")
        table.add_column("vz", style="yellow")
        table.add_column("yaw", style="magenta")
        table.add_column("t_vx", style="red")
        table.add_column("t_vy", style="red")
        table.add_column("t_vz", style="red")
        table.add_column("t_yaw", style="red")

        # Заполняем строки данными каждого объекта
        for i, obj in enumerate(self.simulation_objects):
            # Позиция (state)
            x, y, z, vx, vy, vz = obj.state
            # print(f"obj_channel = ", i, "pos = ", obj.state)
            # Текущий yaw
            yaw = obj.attitude[5]
            # Желаемая скорость
            tvx, tvy, tvz, tyaw = self.t_speed[i]

            table.add_row(
                str(i),
                f"{x:.2f}",
                f"{y:.2f}",
                f"{z:.2f}",
                f"{vx:.2f}",
                f"{vy:.2f}",
                f"{vz:.2f}",
                f"{yaw:.2f}",
                f"{tvx:.2f}",
                f"{tvy:.2f}",
                f"{tvz:.2f}",
                f"{tyaw:.2f}",
            )

        # Инициализируем или обновляем Live
        if self._live is None:
            self._live = Live(table, refresh_per_second=10)
            self._live.start()
        else:
            self._live.update(table)
        # self.print_latest_logs(self.logs, 7, "Таблица с сообщениями")

    def print_latest_logs(
        self, log_dict: dict, n: int = 5, name: str = "Название"
    ) -> None:
        """
        Метод обновляет результаты в таблице логов

        :param log_dict: Словарь с логами заполнения таблицы
        :type log_dict: dict

        :param n: Количество логов из словаря, которые попадут в таблицу
        :type n: int

        :param name: Заголовок таблицы
        :type name: str

        :return: None
        """
        latest_logs = deque(log_dict.items(), maxlen=n)
        table = Table(title=name)
        table.add_column("ID", style="cyan", justify="right")
        table.add_column("Сообщение", style="green")
        for log_id, message in latest_logs:
            table.add_row(str(log_id), message.strip())
        if self._live is None:
            self._live = Live(table, refresh_per_second=20, transient=False)
            self._live.start()
        else:
            self._live.update(table)

    def print_states(self) -> None:
        """
        Выводит в консоль state всех объектов симуляции
        """
        for index, point in enumerate(self.simulation_objects):
            print(
                f"Point: {index} have state: {point.state} and yaw {point.attitude[2]}, yaw_speed {point.attitude[5]}"
            )

    def get_states(self) -> NDArray:
        """
        Отдает вектора состояний всех объектов симуляции
        """
        states = np.zeros(6)
        for point in self.simulation_objects:
            states = np.vstack([states, point.state])
        return states[1:]

    def get_yaws(self) -> NDArray:
        """
        Отдает yaw и сокрость по yaw каждого объекта
        """
        yaws = np.array([0, 0])
        for point in self.simulation_objects:
            yaws = np.vstack([yaws, [point.attitude[2], point.attitude[5]]])
        return yaws

    def swarm_controller(self, dt: float) -> None:
        matrix_signal = self.swarmsolver.solve_for_all(
            state_matrix=self.get_states(),
            target_matrix=self.t_position,
            dt=dt,
        )
        matrix_signal = np.hstack(
            [matrix_signal, np.zeros((matrix_signal.shape[0], 1))]
        )
        self.t_speed = matrix_signal

    def stop(self):
        self.simulation_turn_on = False


class SwarmWriter(SwarmSim):
    def __init__(self, simulation_objects: np.ndarray, dt: float = 0.01, dimension: int = 3, max_speed: float = 1,
                 max_acceleration: float = 10, logger: bool = False, params: Optional[dict] = None):
        super().__init__(simulation_objects, dt, dimension, max_speed, max_acceleration, logger, params)
        self.time: float = 0.
        for sim_object in self.simulation_objects:
            sim_object.trajectory_write = True

    def save_trajectories(self, filename: str = "swarm_trajectory.npz"):
        # Собираем траектории из каждого PointYaw
        all_trajs = [obj.get_trajectory() for obj in self.simulation_objects]
        traj_stack = np.stack(all_trajs, axis=0)
        time_vector = traj_stack[0, :, -1]
        states = np.transpose(traj_stack[:, :, :6], (1, 0, 2))
        print(states.shape)
        np.savez_compressed(
            filename,
            time=time_vector,
            states=states,
            raw_traj=traj_stack
        )
        print(f"Сохранено в {filename}")


    def object_cycle(
        self,
        simulation_object,
        object_channel: int,
        type_of_cycle: str = "while",
        steps: int = 100,
    ) -> None:
        """
        Метод, выполняемый в отдельном потоке для каждого объекта.

        После выполнения шага симуляции поток ждёт, пока все остальные потоки не вызовут sync_barrier.wait().
        Таким образом синхронизуются шаги.
        Дополнение для этого наследника: нет синхронизации по времени
        """
        if type_of_cycle == "while":
            while self.simulation_turn_on:
                if self.swarm_on:
                    self.swarm_controller(self.dt)
                self.velocity_controller()
                simulation_object.step(
                    self.forces[object_channel], self.dt
                )
                self.time += self.dt
                try:
                    self.sync_barrier.wait()
                except threading.BrokenBarrierError:
                    break
        elif type_of_cycle == "for":
            for step in range(steps):
                if object_channel == 0:
                    print(f"Шаг {step + 1}/{steps}")
                if not self.simulation_turn_on:
                    break
                if self.swarm_on:
                    self.swarm_controller(self.dt)
                self.velocity_controller()
                simulation_object.step(self.forces[object_channel], self.dt)
                self.time += self.dt
                try:
                    self.sync_barrier.wait()
                except threading.BrokenBarrierError as e:
                    print(e)
                    break
        else:
            print("Неизвестный тип цикла! Выберите 'while' или 'for'.")


class ScriptedSwarm:
    def __init__(
            self,
            simulation_objects: List[PointYaw],
            script_file: str,
            dt: float = 0.1,
            logger: bool = False
    ):
        # Core simulation parameters
        self.simulation_objects = simulation_objects
        self.num_drones = len(simulation_objects)
        self.dt = dt
        self.logger = logger

        # Thread synchronization
        self.sync_barrier = threading.Barrier(self.num_drones)
        self.simulation_turn_on = False

        # Script parsing
        self.script = self._parse_script(script_file)
        self.current_command_idx = 0
        self.sim_time = 0.0

        # Matrix-based PID for velocity control [vx, vy, vz, yaw_rate]
        self.kp = np.full((self.num_drones, 4), 3.0, dtype=float)
        self.ki = np.zeros((self.num_drones, 4), dtype=float)
        self.kd = np.full((self.num_drones, 4), 0.1, dtype=float)
        self.vel_pid = PIDControllerXd(self.kp, self.ki, self.kd)

    def swarm_controller(self, dt: float):
        # Обновление времени симуляции
        self.time += dt

        # Обработка команд по расписанию
        while (self.current_command < len(self.commands) and (self.time >= self.commands[self.current_command]['time'])):

            self.process_command(self.commands[self.current_command]['command'])
            self.current_command += 1

        # Основная логика управления роем
        super().swarm_controller(dt)

    def _parse_script(self, script_file: str) -> List[Dict[str, Any]]:
        commands: List[Dict[str, Any]] = []
        time_cursor = 0.0
        with open(script_file) as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith('#'):
                    continue
                if s.lower().startswith('sleep'):
                    _, d = s.split()
                    time_cursor += float(d)
                else:
                    commands.append({'time': time_cursor, 'cmd': s})
        return commands

    def _get_target_ids(self, target: str) -> List[int]:
        if target == 'all':
            return list(range(self.num_drones))
        if target.startswith('g:'):
            grp = int(target.split(':')[1])
            return [i for i, obj in enumerate(self.simulation_objects) if obj.group == grp]
        return [int(target)]

    def _execute_command(self, cmd_str: str) -> None:
        parts = cmd_str.split()
        tgt, cmd = parts[0], parts[1].lower()
        ids = self._get_target_ids(tgt)
        if cmd == 'takeoff':
            for i in ids:
                self.simulation_objects[i].state[2] = 1.5
        elif cmd == 'goto' and len(parts) == 6:
            x, y, z, yaw = map(float, parts[2:6])
            for i in ids:
                o = self.simulation_objects[i]
                o.state[:3] = [x, y, z]
                o.attitude[2] = yaw
        elif cmd == 'land':
            for i in ids:
                self.simulation_objects[i].state[2] = 0.0
        elif cmd == 'setgroup' and len(parts) == 3:
            newg = int(parts[2])
            for i in ids:
                self.simulation_objects[i].group = newg
            cfg = {str(j): obj.group for j, obj in enumerate(self.simulation_objects)}
            with open('groups.json', 'w') as f:
                json.dump(cfg, f)

    def object_cycle(self, idx: int) -> None:
        obj = self.simulation_objects[idx]
        while self.simulation_turn_on:
            # only thread 0 triggers script
            if idx == 0:
                while (self.current_command_idx < len(self.script) and
                       self.sim_time >= self.script[self.current_command_idx]['time']):
                    c = self.script[self.current_command_idx]['cmd']
                    print(f"[{self.sim_time:.1f}s] Exec: {c}")
                    self._execute_command(c)
                    self.current_command_idx += 1
                self.sim_time += self.dt

            # Build arrays for PID call
            target_vel = np.zeros((self.num_drones, 4), dtype=float)
            current_vel = np.zeros((self.num_drones, 4), dtype=float)
            for j, o in enumerate(self.simulation_objects):
                # extract from state: vx, vy, vz
                vx, vy, vz = o.state[3:6]
                yaw_rate = o.attitude[5]
                current_vel[j] = [vx, vy, vz, yaw_rate]
                # target_vel remains zero or could be set from commands

            # compute forces in batch
            forces = self.vel_pid.compute_control(target_vel, current_vel, self.dt)

            # apply for this drone
            fx, fy, fz, fry = forces[idx]
            # translational step
            o_des = obj
            drag = -o_des.drag_coefficient * np.array([vx, vy, vz])
            total_force = np.array([fx, fy, fz]) + drag
            # update state via simple Euler (or can call o_des.step)
            acc = total_force / o_des.mass
            o_des.state[3:6] += acc * self.dt
            o_des.state[:3] += o_des.state[3:6] * self.dt
            # yaw dynamics
            o_des.attitude[5] += fry * self.dt
            o_des.attitude[2] += o_des.attitude[5] * self.dt

            if self.logger and idx == 0:
                print(f"Time {self.sim_time:.2f}")

            try:
                self.sync_barrier.wait()
            except threading.BrokenBarrierError:
                break

    def start_simulation_for(self, steps: int) -> None:
        self.simulation_turn_on = True
        # launch threads
        self.threading_list: List[threading.Thread] = []
        for i in range(self.num_drones):
            t = threading.Thread(target=self.object_cycle, args=(i,), daemon=True)
            self.threading_list.append(t)
            t.start()
        # run for fixed steps
        for _ in range(steps):
            time.sleep(self.dt)
        self.simulation_turn_on = False

    def save_trajectories(self, filename: str = 'trajectory.npz') -> None:
        trajs = [o.get_trajectory() for o in self.simulation_objects]
        np.savez_compressed(filename, trajectories=trajs)

# Usage example
def main():
    number_of_objects = 16
    script = '/home/user/code/pion/scripts/mission_script.txt'
    output = 'scripted_swarm_data.npz'
    simulation_duration = 60.0  # seconds

    # Create swarm objects externally
    objs = create_objects_Point_yaw(int(np.sqrt(number_of_objects)), [-5,5], [-5,5], 0.2)
    # Assign even/odd groups
    for idx, o in enumerate(objs):
        o.group = idx % 2

    # Initialize runner
    runner = SimulationRunner(
        num_drones=number_of_objects,
        script_file=script,
        output_file=output,
        sim_time=simulation_duration
    )

    # Override objects and start
    runner.objects = objs
    runner.run()



class SimulationRunner:
    def __init__(
            self,
            num_drones: int = 16,
            script_file: str = "mission.txt",
            output_file: str = "trajectory.npz",
            sim_time: float = 10.0,
    ):
        side = int(np.sqrt(num_drones))
        self.objects = create_objects_Point_yaw(side)

        self.sim = ScriptedSwarm(
            script_file=script_file,
            simulation_objects=self.objects,
            dt=0.1,
            logger=False
        )

        self.output_file = output_file
        self.sim_time = sim_time

    def run(self):
        self.sim.start_simulation_for(int(self.sim_time / self.sim.dt))

        # Ожидаем завершения потоков
        for thread in self.sim.threading_list:
            thread.join()

        self.sim.save_trajectories(self.output_file)
        print("Simulation completed and trajectory saved")

if __name__ == '__main__':
    main()
# if __name__ == "__main__":
#     # Параметры симуляции
#     number_of_objects = 16
#     script_file = "/home/user/code/pion/scripts/mission_script.txt"
#     bounds = {
#         'x': [-5, 5],
#         'y': [-5, 5],
#         'z': [0, 3]
#     }
#
#     # Создание объектов роя
#     swarm_objects = create_objects_Point_yaw(
#         number=int(np.sqrt(number_of_objects)),
#         linspace_x=bounds['x'],
#         linspace_y=bounds['y'],
#         z_offset=0.2
#     )
#
#     # Инициализация ScriptedSwarm с привязкой к скрипту
#     sim = ScriptedSwarm(
#         num_drones=number_of_objects,
#         script_file=script_file,
#         simulation_objects=swarm_objects,
#         dt=0.1,
#         logger=True
#     )
#
#     # Настройка параметров роя
#     sim.swarm_on = True
#
#     # Запуск симуляции на 60 секунд
#     sim.start_simulation_for(int(60 / sim.dt))
#
#     # Ожидание завершения всех потоков
#     for thread in sim.threading_list:
#         thread.join()
#
#     # Сохранение результатов
#     sim.save_trajectories("scripted_swarm_data.npz")
#     print("Scripted simulation completed successfully")

    # number_of_objects = 30
    # swarmsim = SwarmWriter(
    #     create_objects_Point_yaw(
    #         int(np.sqrt(number_of_objects)), [-5, 5], [-5, 5], z_offset=0
    #     ),
    #     dt=0.1,
    #     logger=True,
    # )
    # swarmsim.swarm_on = True
    # swarmsim.start_simulation_for(1000)
    # for th in swarmsim.threading_list:
    #     th.join()
    # swarmsim.save_trajectories("my_swarm_data.npz")
