from typing import Union

import numpy as np
import numpy.typing as npt


class PIDController:
    """
    Класс реализует ПИД-регулятор (пропорционально-интегрально-дифференциальный регулятор).

    Поддерживает управление как для одной оси, так и для нескольких осей одновременно.
    Коэффициенты могут быть заданы как скалярными значениями, так и векторами для каждой оси.

    :param kp: Пропорциональный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
    :param ki: Интегральный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
    :param kd: Дифференциальный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
    """

    def __init__(
        self,
        kp: Union[float, int, list, npt.NDArray[np.float64]],
        ki: Union[float, int, list, npt.NDArray[np.float64]],
        kd: Union[float, int, list, npt.NDArray[np.float64]],
    ):
        """
        Инициализация ПИД-регулятора с коэффициентами пропорциональности, интеграла и дифференциала.

        :param kp: Пропорциональный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
        :type kp: Union[float, int, list, np.ndarray[float64]]
        :param ki: Интегральный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
        :type ki: Union[float, int, list, np.ndarray[float64]]
        :param kd: Дифференциальный коэффициент(ы). Может быть скалярным (float, int) или вектором (list, np.ndarray).
        :type kd: Union[float, int, list, np.ndarray[float64]]

        Этот класс поддерживает управление для нескольких осей. Коэффициенты могут быть как скалярными значениями,
        так и векторами, где каждый элемент соответствует своей оси.
        """

        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)

        # Переменные для хранения предыдущих значений
        self.previous_error = None
        self.integral = np.zeros_like(self.kp, dtype=np.float64)

    def compute_control(
        self,
        target_position: Union[float, np.ndarray, list],
        current_position: Union[float, np.ndarray, list],
        dt: Union[float, int] = 0.0,
    ) -> Union[float, np.ndarray]:
        """
        Вычисляет управляющий сигнал на основе ПИД-регулирования для произвольного количества осей.

        :param target_position: Желаемое положение для каждой оси. Может быть скалярным значением или вектором.
        :type target_position: Union[float, np.ndarray, list]
        :param current_position: Текущее положение для каждой оси. Может быть скалярным значением или вектором.
        :type current_position: Union[float, np.ndarray, list]
        :param dt: Шаг времени, используемый для вычисления интегральной и дифференциальной частей. По умолчанию 0.
        :type dt: Union[float, int]
        :return: Рассчитанный управляющий сигнал, который может быть как скалярным значением, так и вектором.
        :rtype: Union[float, np.ndarray]

        Функция вычисляет пропорциональную, интегральную и дифференциальную составляющие для регулирования
        управляющего сигнала на основе ошибки между целевым и текущим положениями.
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
        if dt == 0.0:
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
