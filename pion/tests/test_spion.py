from pion import Spion
import numpy as np
from pion.cython_pid import PIDController

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
        current_position = np.array(np.array([0, 0, 0]), dtype=np.float64)
        # Отдельно запишем целевую скорость
        target_speed = t_speed[0:3]
        # Создание отдельного объекта ПИД
        pid_velocity_controller = PIDController(np.array([3., 3., 3.], dtype=np.float64),
                                                np.array([0., 0., 0.], dtype=np.float64),
                                                np.array([0.1, 0.1, 0.1], dtype=np.float64))
        # Создание экземпляра Spion с отключенным message_handler
        spion = Spion(start_message_handler_from_init=False)
        spion._pid_velocity_controller = PIDController(*spion.velocity_pid_matrix)
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
            dt=0.1)
        # Сравнение управляющего сигнала с ПИД spion и проверочного ПИД
        assert np.all(np.equal(force_after_one_step, signal))

    def test_position_controller(self):
        """
        Тестирование контроллера позиции.
        """
        # Назначение целевой координаты
        target_xyz = np.array([1, 2, 3], dtype=np.float64)
        # Назначение текущей позиции
        current_position = np.array(np.array([0, 0, 0]), dtype=np.float64)
        # Создание отдельного объекта ПИД
        pid_position_controller = PIDController(np.array([1., 1., 1.], dtype=np.float64),
                                                np.array([0., 0., 0.], dtype=np.float64),
                                                np.array([0., 0., 0.], dtype=np.float64))
        # Создание экземпляра Spion с отключенным message_handler
        spion = Spion(start_message_handler_from_init=False)
        spion._pid_velocity_controller = PIDController(*spion.velocity_pid_matrix)
        spion._pid_position_controller = PIDController(*spion.position_pid_matrix)
        spion.max_speed = float("inf")

        spion.position_controller(target_xyz)
        # Один шаг моделирования симулятора Spion
        spion._step_messege_handler()
        # Вычисление управлявляющего сигнала с нашего ПИД регулятора
        signal = pid_position_controller.compute_control(
            target_position=target_xyz,
            current_position=current_position,
            dt=0.1)
        # Сравнение управляющего сигнала с ПИД spion и проверочного ПИД
        assert np.all(np.equal(spion.t_speed[0:3], signal))

        
        
                


