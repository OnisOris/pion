import numpy as np
from numpy.typing import NDArray


class Trajectory:
    """
    Класс для хранения и управления траекторией движения.

    Хранит траекторию в виде матрицы размером nxm, где n - количество точек,
    m - количество параметров (длина list_of_names_columns).
    """

    def __init__(self, list_of_names_columns: list[str]):
        """
        Специальный класс для записи и хранения траектории размером nx[len(list_of_names_columns)], n - количество точек.

        :param list_of_names_columns: названия колонн
        :type list_of_names_columns: list[str]
        """
        self.trajectory = np.zeros((len(list_of_names_columns),))
        self.columns = list_of_names_columns
        self.stopped = False

    def vstack(self, vstack_array: NDArray[np.float64]) -> None:
        """
        Метод объединяет входящие вектора с матрицей trajectory

        :param vstack_array: массив размерности len(list_of_names_columns)
        :type vstack_array: NDArray[np.float64]
        :return: None
        :rtype: None
        """
        if not self.stopped:
            self.trajectory = np.vstack([self.trajectory, vstack_array])

    def stop(self) -> None:
        """
        Остановка записи траектории
        """
        self.stopped = True

    def get_trajectory(self) -> NDArray[np.float64]:
        """
        Метод возвращает записанную траекторию

        :return: NDArray[np.float64]
        :rtype: None
        """
        if not self.trajectory.shape == (len(self.columns),):
            return self.trajectory[1:]
        else:
            return self.trajectory
