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
