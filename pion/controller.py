
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


# Пример использования P-регулятора
if __name__ == "__main__":
    # Инициализация P-регулятора с коэффициентами для каждой оси (произвольное количество осей)
    # Здесь kp — вектор коэффициентов для каждой оси
    p_controller = PController(kp=[1.0, 1.0, 1.0, 0.5])

    # Целевая позиция (может быть сколько угодно координат)
    target_position = [10.0, 5.0, 3.0, 8.0]

    # Текущая позиция дрона (с таким же количеством координат)
    current_position = [8.0, 4.0, 2.5, 6.0]

    # Вычисляем управляющие скорости по каждой оси
    control_signal = p_controller.compute_control(target_position, current_position)

    print(f"Управляющие сигналы: {control_signal}")
