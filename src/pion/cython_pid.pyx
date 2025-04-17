import numpy as np
cimport numpy as np

cdef class PIDController:
    # Объявление атрибутов как указателей или объектов (а не буферов)
    cdef object kp, ki, kd, integral, previous_error

    def __init__(self, np.ndarray[np.float64_t] kp, np.ndarray[np.float64_t] ki, np.ndarray[np.float64_t] kd):
        # Инициализация атрибутов как np.ndarray
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros_like(self.kp, dtype=np.float64)
        self.previous_error = np.zeros_like(self.kp, dtype=np.float64)

    def compute_control(self, np.ndarray[np.float64_t] target_position, np.ndarray[np.float64_t] current_position, double dt):
        cdef np.ndarray[np.float64_t] error, p_term, i_term, d_term, derivative, control_signal

        error = target_position - current_position
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral

        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = np.zeros_like(error)
        d_term = self.kd * derivative
        self.previous_error = error

        control_signal = p_term + i_term + d_term
        return control_signal

cdef class PIDControllerXd:
    # Объявление атрибутов как указателей или объектов (а не буферов)
    cdef object kp, ki, kd, integral, previous_error

    def __init__(self,
                 np.ndarray[np.float64_t, ndim = 2] kp,
                 np.ndarray[np.float64_t, ndim = 2] ki,
                 np.ndarray[np.float64_t, ndim = 2] kd):
        # Инициализация атрибутов как np.ndarray
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = np.zeros_like(self.kp, dtype=np.float64)
        self.previous_error = np.zeros_like(self.kp, dtype=np.float64)

    def compute_control(self,
                        np.ndarray[np.float64_t, ndim = 2] target_position,
                        np.ndarray[np.float64_t, ndim = 2] current_position,
                        double dt):
        cdef np.ndarray[np.float64_t, ndim = 2] error, p_term, i_term, d_term, derivative, control_signal

        error = target_position - current_position
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral

        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = np.zeros_like(error)
        d_term = self.kd * derivative
        self.previous_error = error

        control_signal = p_term + i_term + d_term
        return control_signal




cdef class AdaptiveController:
    cdef object kp, ki, kd, integral, previous_error
    cdef object error_history, max_adjustment, learning_rate

    def __init__(self,
                 np.ndarray[np.float64_t] initial_kp,
                 np.ndarray[np.float64_t] initial_ki,
                 np.ndarray[np.float64_t] initial_kd,
                 double learning_rate=0.01,
                 double max_adjustment=0.1):
        self.kp = initial_kp.copy()
        self.ki = initial_ki.copy()
        self.kd = initial_kd.copy()
        self.integral = np.zeros_like(initial_kp, dtype=np.float64)
        self.previous_error = np.zeros_like(initial_kp, dtype=np.float64)
        self.error_history = np.zeros((10, initial_kp.shape[0]))  # История ошибок
        self.learning_rate = learning_rate
        self.max_adjustment = max_adjustment

    def compute_control(self,
                        np.ndarray[np.float64_t] target_position,
                        np.ndarray[np.float64_t] current_position,
                        double dt):
        cdef np.ndarray[np.float64_t] error = target_position - current_position
        cdef np.ndarray[np.float64_t] derivative

        # Обновляем историю ошибок
        self.error_history = np.roll(self.error_history, shift=1, axis=0)
        self.error_history[0] = error

        # Адаптация коэффициентов
        delta_kp = self.learning_rate * np.mean(self.error_history, axis=0) * error
        delta_kp = np.clip(delta_kp, -self.max_adjustment, self.max_adjustment)
        self.kp += delta_kp

        # Вычисление управления
        self.integral += error * dt
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = np.zeros_like(error)

        control_signal = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        self.previous_error = error.copy()
        return control_signal
