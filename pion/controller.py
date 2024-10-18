
import numpy as np

class PController:
    def __init__(self, kp):
        """
        Инициализация P-регулятора с коэффициентом пропорциональности.
        kp может быть как скалярным значением, так и вектором коэффициентов для каждой оси.
        """
        self.kp = np.array(kp)

    def compute_control(self, target_position, current_position):
        """
        Рассчитывает управляющую скорость для произвольного количества осей.
        target_position и current_position — векторы numpy с одинаковым количеством элементов.
        """
        # Преобразуем в numpy массивы для работы с векторами
        target_position = np.array(target_position)
        current_position = np.array(current_position)

        # Вычисляем ошибку: разница между целевой и текущей позицией
        error = target_position - current_position

        # Применяем P-регулятор для каждой оси
        control_signal = self.kp * error

        return control_signal


