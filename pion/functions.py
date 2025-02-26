import numpy as np
from typing import Union
import numpy.typing as npt
from pymavlink import mavutil


def normalization(vector: np.ndarray, length: float = 1.) -> np.ndarray:
    """
    Функция возвращает нормированный вектор заданной длины

    :param vector: Вектор
    :type vector: ndarray

    :param length: Длина вектора
    :type length: float

    :return: Нормированный вектор длины length
    :rtype: np.ndarray
    """
    return np.array(vector) / np.linalg.norm(vector) * length

def vector_rotation2(vector: np.ndarray, angle: float) -> np.ndarray:
    """
    Вращение вектора по часовой стрелке

    :param vector: Координаты вектора размерностью 2
    :type vector: np.ndarray

    :param angle: Угол в радианах
    :type angle: float

    :return: Повернутый вектор на ange градусов
    :rtype: np.ndarray
    """
    matrix_rotation = np.array([[np.cos(angle), np.sin(angle)],
                                [-np.sin(angle), np.cos(angle)]])
    new_vector = matrix_rotation.dot(vector)
    return new_vector

def create_connection(
        connection_method: str,
        ip: str,
        port: Union[int, str]
) -> mavutil.mavfile:
    """
    Создаёт MAVLink соединение.

    :param connection_method: Метод соединения, например, 'udp', 'tcp', или другой.
    :type connection_method: str

    :param ip: IP-адрес для соединения, например '127.0.0.1'
    :type ip: str

    :param port: Порт для соединения. Может быть целым числом или строкой.
    :type port: Union[int, str]

    :return: Возвращает объект mav_socket, который представляет MAVLink соединение.
    :rtype: mavutil.mavfile
    """
    mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (connection_method, ip, port))
    return mav_socket


def update_array(
        arr: Union[list, npt.NDArray[Union[float, list, npt.NDArray[np.float64]]]],
        new_value: Union[float, list, npt.NDArray[np.float64]]
) -> npt.NDArray[np.float64]:
    """
    Сдвигает элементы массива и вставляет новое значение в начало.

    :param arr: Входной массив или список для обновления. Может быть списком,
                numpy-массивом с числами типа float, либо вложенной структурой
                из списков или массивов.
    :type arr: Union[list, npt.NDArray[Union[float, list, npt.NDArray[np.float64]]]]

    :param new_value: Значение, которое будет помещено в начало массива.
    :type new_value: Union[float, list, npt.NDArray[np.float64]]

    :return: Обновлённый массив с новым значением в начале.
    :rtype: Union[list, npt.NDArray[np.float64]]
    """
    arr = np.roll(arr, 1, axis=0)
    arr[0] = new_value
    return arr


def update_vector(
        vector: Union[list, npt.NDArray[np.float64]],
        new_value: float
) -> npt.NDArray[np.float64]:
    """
    Сдвигает элементы вектора и вставляет новое значение (скаляр) в начало.

    :param vector: Входной вектор для обновления. Может быть списком или numpy-массивом с числами типа float.
    :type vector: Union[list, npt.NDArray[np.float64]]

    :param new_value: Скалярное значение, которое будет помещено в начало вектора.
    :type new_value: float

    :return: Обновлённый вектор с новым значением в начале.
    :rtype: npt.NDArray[np.float64]
    """
    # Преобразуем входные данные в numpy-массив, если это список
    vector = np.asarray(vector, dtype=np.float64)

    # Сдвигаем вектор на 1 позицию вправо
    vector = np.roll(vector, 1)

    # Вставляем новое значение в начало
    vector[0] = new_value

    return vector


def compare_with_first_row(
        matrix: Union[list, npt.NDArray[np.float64]],
        atol: float = 8e-2) -> bool:
    """
    Проверяет, являются ли все строки матрицы близкими к первой строке в пределах допуска.

    :param matrix: Входная матрица, представленная списком или numpy-массивом.
    :type matrix: Union[list, npt.NDArray[np.float64]]

    :param atol: Абсолютная погрешность для сравнения строк. Значение по умолчанию равно 1e-2.
    :type atol: float

    :return: Возвращает True, если все строки матрицы близки к первой строке в пределах указанной погрешности, иначе False.
    :rtype: bool
    """
    first_row = matrix[0]
    return np.all([np.allclose(first_row, row, atol=atol) for row in matrix[1:]])


def vector_reached(target_vector: Union[list, npt.NDArray[np.float64]],
                   current_point_matrix: Union[list, npt.NDArray[np.ndarray]],
                   accuracy: Union[int, float] = 5e-2) -> bool:
    """
    Функция сравнивает текующую позицию с целевой позицией, возвращает True в пределах погрешности accuracy
    :param target_vector: целевой вектор
    :type: Union[list, npt.NDArray[np.float64]]
    :param current_point_matrix: текущий вектор состояния дрона
    :type: Union[list, npt.NDArray[np.ndarray]]
    :param accuracy: Погрешность целевой точки
    :type: Union[int, float]
    :return: bool
    """
    matrix = np.vstack([target_vector, current_point_matrix])
    if compare_with_first_row(matrix, accuracy):
        return True
    else:
        return False


def scalar_reached(target_vector: Union[list, npt.NDArray[np.float64]],
                   current_point_matrix: Union[list, npt.NDArray[np.ndarray]],
                   accuracy: Union[int, float] = 5e-2) -> bool:
    """
    Функция сравнивает текующую позицию с целевой позицией, возвращает True в пределах погрешности accuracy
    :param target_vector: целевой вектор
    :type: Union[list, npt.NDArray[np.float64]]
    :param current_point_matrix: текущий вектор состояния дрона
    :type: Union[list, npt.NDArray[np.ndarray]]
    :param accuracy: Погрешность целевой точки
    :type: Union[int, float]
    :return: bool
    """
    matrix = np.hstack([target_vector, current_point_matrix])
    if compare_with_first_row(matrix, accuracy):
        return True
    else:
        return False
