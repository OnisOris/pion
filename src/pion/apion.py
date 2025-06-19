import asyncio
import select
import time

import numpy as np
from pymavlink import mavutil

from pionfunc.functions import vector_reached

from .controller import PIDController
from .pion import Pion


class Apion(Pion):
    """
    Асинхронная версия Pion
    """

    async def message_handler(self, combine_system: int = 0) -> None:
        """
        Асинхронно обрабатывает сообщения от дрона и отправляет heartbeat, обновляя координаты дрона.

        :param combine_system: Определяет, с каких источников будут считываться данные:
                                0 — только локус, 1 — локус и оптика, 2 — только оптика.
        :type combine_system: int
        :return: None
        """
        src_component_map = {
            0: 1,  # Только локус
            1: None,  # Локус и оптика (неважно, откуда приходит)
            2: 26,  # Только оптика
        }
        src_component = src_component_map.get(combine_system)

        while self.message_handler_flag:
            if not self.__is_socket_open.is_set():
                break

            self.heartbeat()

            # Асинхронно ждем сообщения
            rlist, _, _ = await asyncio.to_thread(
                select.select,
                [self.mavlink_socket.port.fileno()],
                [],
                [],
                self.period_message_handler,
            )
            if rlist:
                # Асинхронно получаем сообщение
                self._msg = await asyncio.to_thread(
                    self.mavlink_socket.recv_msg
                )
                if self._msg is not None:
                    self._process_message(self._msg, src_component)
                else:
                    print("Received empty message or timeout.")
            if self.check_attitude_flag:
                print("Attitude write")
                self.attitude_write()
            await asyncio.sleep(self.period_message_handler)

    async def send_speed(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float,
    ) -> None:
        """
        Асинхронно задает вектор скорости дрону. Отсылать необходимо в цикле.

        :param vx: Скорость по оси X (м/с)
        :param vy: Скорость по оси Y (м/с)
        :param vz: Скорость по оси Z (м/с)
        :param yaw_rate: Скорость поворота по yaw (рад/с)
        :return: None
        """
        # Преобразуем ENU координаты в NED
        vx, vy, vz = vy, vx, -vz  # Конвертация координат

        # Маска, указывающая, что мы отправляем только скорость и yaw_rate
        mask = 0b0000_01_0_111_000_111

        # Полная команда с отправкой скорости и yaw
        await asyncio.to_thread(
            self._send_position_target_local_ned,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            0,
            0,
            0,  # Позиция игнорируется
            vx,
            vy,
            vz,  # Скорости
            0,
            0,
            0,  # Ускорения
            0,  # yaw
            yaw_rate,  # yaw rate
            mavlink_send_number=1,
        )

    async def v_while(self) -> None:
        """
        Асинхронный метод, которая отправляет вектор скорости в цикле.

        :param ampl: множитель вектора скорости
        :type ampl: Union[float, int]
        :return: None
        """
        while self.speed_flag:
            # Увеличиваем вектор скорости на амплитуду
            t_speed = self.t_speed

            # Отправляем скорость дрону
            await self.send_speed(
                t_speed[0], t_speed[1], t_speed[2], t_speed[3]
            )
            await asyncio.sleep(self.period_send_speed)

    async def main(self) -> None:
        """
        Главная асинхронный метод для запуска обработки сообщений и отправки скоростей.

        :return: None
        """
        # Запуск обработчика сообщений
        task1 = asyncio.create_task(self.message_handler(combine_system=0))

        # Запуск цикла отправки скоростей
        task2 = asyncio.create_task(self.v_while())

        # Ожидание завершения задач (в зависимости от вашей логики)
        await task1
        await task2

    def stop(self) -> None:
        """
        Останавливает все асинхронные задачи внутри приложения.

        :return: None
        """
        self.speed_flag = False
        self.check_attitude_flag = False
        self.message_handler_flag = False

    async def set_v_async(self) -> None:
        """
        Асинхронно запускает цикл, который вызывает функцию v_while() для параллельной отправки вектора скорости.

        :return: None
        """
        self.speed_flag = True
        await self.v_while()

    async def goto_from_outside(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        accuracy: float = 8e-2,
    ) -> None:
        """
        Асинхронный метод для перемещения дрона к указанной точке с учетом управления yaw.

        :param x: координата по x
        :type x: float
        :param y: координата по y
        :type y: float
        :param z:  координата по z
        :type z: float
        :param yaw: координата по yaw
        :type yaw: float
        :param accuracy: Погрешность целевой точки
        :type accuracy: float

        :return: None
        """
        await self.goto_yaw(yaw)
        pid_controller = PIDController(*self.position_pid_matrix)
        point_reached = False
        dt = time.time()  # Для расчета временного шага
        while not point_reached:
            dt = time.time() - dt  # Расчет временного шага
            point_reached = vector_reached(
                [x, y, z], self.position[0:3], accuracy=accuracy
            )

            # Рассчитываем управляющие воздействия на основе текущей позиции
            control = pid_controller.compute_control(
                [x, y, z], self.position[0:3]
            )
            control_clipped = np.clip(
                control, -self.max_speed, self.max_speed
            )  # Ограничиваем максимальную скорость

            # Обновляем t_speed, добавляем нулевой yaw
            self.t_speed = np.hstack([control_clipped, 0])

            # Ждем некоторое время перед следующим расчетом
            await asyncio.sleep(self.period_send_speed)

        # Останавливаем движение после достижения точки
        self.t_speed = np.array([0, 0, 0, 0])

    async def goto_yaw(self, yaw: float = 0, accuracy: float = 0.087) -> None:
        """
        Асинхронный метод для поворота дрона на указанный угол yaw.

        :param yaw:  координата по yaw (радианы)
        :type yaw: float
        :param accuracy: Погрешность целевой точки
        :type accuracy: float
        :return: None
        """
        pid_controller = PIDController(1, 0, 1)
        point_reached = False
        while not point_reached:
            current_yaw = self.attitude[2]
            point_reached = vector_reached(yaw, current_yaw, accuracy=accuracy)
            # Рассчитываем yaw скорость
            yaw_control = pid_controller.compute_control(yaw, current_yaw)
            yaw_speed = np.clip(
                yaw_control, -self.max_speed, self.max_speed
            )  # Ограничиваем скорость yaw
            # Обновляем t_speed только для yaw
            self.t_speed = np.array([0, 0, 0, -yaw_speed])
            await asyncio.sleep(self.period_send_speed)
        # Останавливаем yaw после достижения цели
        self.t_speed = np.array([0, 0, 0, 0])
