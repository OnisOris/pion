import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        """
        Инициализация ПИД-регулятора с коэффициентами пропорциональности, интеграла и дифференциала.
        kp, ki и kd могут быть скалярными значениями или векторами коэффициентов для каждой оси.
        dt — шаг по времени для вычисления интегральной и дифференциальной частей.
        """
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)

        # Переменные для хранения предыдущих значений
        self.previous_error = None
        self.integral = np.zeros_like(self.kp)

    def compute_control(self, target_position: float | np.ndarray | list, current_position: float | np.ndarray | list, dt: float | int = 0.) -> float | np.ndarray:
        """
        Рассчитывает управляющую скорость для произвольного количества осей на основе ПИД-регулирования.
        target_position и current_position — векторы numpy с одинаковым количеством элементов.
        """
        target_position = np.array(target_position)
        current_position = np.array(current_position)

        # Вычисляем ошибку
        error = target_position - current_position

        # Пропорциональная составляющая
        p_term = self.kp * error

        # Интегральная составляющая (с накоплением)
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Дифференциальная составляющая (разность ошибок)
        if dt == 0.:
            derivative = 0
        elif self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = np.zeros_like(error)
        d_term = self.kd * derivative

        # Сохраняем текущую ошибку для следующего расчета
        self.previous_error = error

        # Итоговое управляющее воздействие
        control_signal = p_term + i_term + d_term

        return control_signal



