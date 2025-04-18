import numpy as np

from pion.cython_pid import PIDController, PIDControllerXd
from pion.spion import Spion


class TestSpion:
    def test_position(self):
        """
        Тетсирование геттера position
        """
        position = np.array([1, 0, 1, 0, 1, 0])
        spion = Spion(position=position, start_message_handler_from_init=False)
        assert np.all(np.equal(position, spion.position))

    def test_attitude(self):
        """
        Тестирование геттера attitude
        """
        attitude = np.array([1, 0, 1, 0, 1, 0])
        spion = Spion(attitude=attitude, start_message_handler_from_init=False)
        assert np.all(np.equal(attitude, spion.attitude))

    def test_velocity_controller(self):
        """
        Тестирование контроллера скорости.

        Создается отдельно регулятор со стандартными коэффициентами ПИД из конструктора и
        проверяется управляеющее воздействие первой итерации
        """
        # Назначение целевой скорости
        t_speed = np.array([1, 2, 3, 4], dtype=np.float64)
        # Назначение текущей позиции
        current_position = np.array(np.array([0, 0, 0, 0]), dtype=np.float64)
        # Отдельно запишем целевую скорость
        target_speed = t_speed

        # Создание экземпляра Spion с отключенным message_handler
        spion = Spion(start_message_handler_from_init=False)
        # Создание отдельного объекта ПИД
        pid_velocity_controller = PIDController(*spion.velocity_pid_matrix)
        spion._pid_velocity_controller = PIDController(
            *spion.velocity_pid_matrix
        )
        # Присвоем spion целевую скорость
        spion.t_speed = t_speed
        # Один шаг моделирования симулятора Spion
        spion._step_messege_handler()
        # Забираем управляюшее воздействие
        force_after_one_step = spion.forces[0]
        # Вычисление управлявляющего сигнала с нашего ПИД регулятора
        signal = pid_velocity_controller.compute_control(
            target_position=target_speed,
            current_position=current_position,
            dt=0.1,
        )
        # Сравнение управляющего сигнала с ПИД spion и проверочного ПИД
        assert np.all(
            np.equal(
                force_after_one_step,
                np.clip(
                    signal, -spion.max_acceleration, spion.max_acceleration
                ),
            )
        )

    def test_position_controller(self):
        """
        Тестирование контроллера позиции.
        """
        # Назначение целевой координаты
        target_xyz = np.array([1, 2, 3, 4], dtype=np.float64)
        # Назначение текущей позиции
        current_position = np.array(np.array([0, 0, 0, 0]), dtype=np.float64)

        # Создание экземпляра Spion с отключенным message_handler
        spion = Spion(start_message_handler_from_init=False)
        # Создание отдельного объекта ПИД
        pid_position_controller = PIDController(*spion.position_pid_matrix)
        spion._pid_velocity_controller = PIDController(
            *spion.velocity_pid_matrix
        )
        spion._pid_position_controller = PIDController(
            *spion.position_pid_matrix
        )
        spion.max_speed = float("inf")
        spion.position_controller(target_xyz)
        # Один шаг моделирования симулятора Spion
        spion._step_messege_handler()
        # Вычисление управлявляющего сигнала с нашего ПИД регулятора
        signal = pid_position_controller.compute_control(
            target_position=target_xyz,
            current_position=current_position,
            dt=0.1,
        )
        # Сравнение управляющего сигнала с ПИД spion и проверочного ПИД
        assert np.all(
            np.equal(
                spion.t_speed,
                np.clip(signal, -spion.max_speed, spion.max_speed),
            )
        )


class TestSpionXd:
    def test_position_controller_Xd(self):
        """
        Тестирование контроллера позиции в многомерном режиме с использованием PIDControllerXd.
        """
        count_of_objects = 2

        # Задаем матрицы коэффициентов для ПИД. Размер матрицы должен совпадать с размерностями
        # целевых и текущих позиций (в данном примере shape=(2,4)).
        kp = np.array(
            [[3.0, 3.0, 3.0, 3.0], [3.0, 3.0, 3.0, 3.0]], dtype=np.float64
        )
        ki = np.array(
            [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]], dtype=np.float64
        )
        kd = np.array(
            [[0.1, 0.1, 0.1, 0.1], [0.1, 0.1, 0.1, 0.1]], dtype=np.float64
        )

        # Целевая позиция (shape=(2,4))
        target_xyz = np.array([[1, 2, 3, 4], [1, 2, 3, 4]], dtype=np.float64)
        # Текущая позиция (shape=(2,4))
        current_position = np.array(
            [[0, 0, 0, 0], [1, 1, 1, 1]], dtype=np.float64
        )

        # Создание экземпляра нового класса PIDControllerXd
        pid_controller_xd = PIDControllerXd(kp, ki, kd)

        # Вычисление управляющего сигнала
        signal = pid_controller_xd.compute_control(
            target_position=target_xyz,
            current_position=current_position,
            dt=0.1,
        )

        print("Control Signal:\n", signal)
        # Проверяем, что возвращаемая матрица имеет ожидаемую форму (2,4)
        assert signal.shape == (count_of_objects, 4), (
            "Размер сигнала не соответствует ожидаемому (2,4)"
        )


if __name__ == "__main__":
    test = TestSpionXd()
    test.test_position_controller_Xd()
