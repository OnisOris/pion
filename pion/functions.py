import numpy as np
from typing import Union
import numpy.typing as npt
from pymavlink import mavutil
import socket
from pion.annotation import Array6

def get_local_ip():
    """
    Определяет локальный IP-адрес, используя временный UDP-сокет.
    При неудаче возвращает '127.0.0.1'.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Подключаемся к внешнему адресу (8.8.8.8) для определения локального IP
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "127.0.0.1"
    finally:
        s.close()
    return local_ip

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

def limit_acceleration(current_velocity: np.ndarray,
                        target_velocity: np.ndarray,
                        max_acceleration: float) -> np.ndarray:
    """
    Метод ограничения максимального ускорения.
    Работает с векторами любого рамера.
    :param current_velocity: массив скоростей
    :type current_velocity: np.ndarray

    :param target_velocity: целевая скорость
    :type target_velocity: np.ndarray
    :param max_acceleration: предел по максимальному ускорению
    :type max_acceleration: float

    :return: максимальное изменение скорости
    :rtype: np.ndarray
    """
    change = target_velocity - current_velocity
    norm = np.linalg.norm(change)
    if norm > max_acceleration:
        change = change / norm * max_acceleration
    return current_velocity + change

def limit_speed(velocity: np.ndarray,
                 max_speed: float) -> np.ndarray:
    """
    Метод для ограничения скорости.
    В диапазоне (0.03, 0.45) скорость делится на 1.4.
    В остальном ограничение идет по max_speed.
    
    :param velocity: Ограничеваемая скорость
    :type velocity: np.ndarray

    :return: np.ndarray
    :rtype: np.ndarray
    """
    norm = np.linalg.norm(velocity)
    # Ограничение скорости возможно стоит ограничить определенной функцией
    if 0.03 < norm < 0.45:
        return velocity / 1.4
    elif norm < 0.06:
        return np.zeros_like(velocity)
    if norm > max_speed:
        return velocity / norm * max_speed
    return velocity

def compute_swarm_velocity(state_vector: Array6,
                           env: dict,
                           target_point: np.ndarray = np.array([0, 0]),
                           safety_radius: float = 1,
                           max_speed: float = 0.4) -> np.ndarray:
    """
    Вычисляет желаемый вектор скорости для локального дрона на основе информации из env.
    Логика:
        - Attraction: направлен от текущей позиции к средней позиции остальных дронов.
        - Repulsion: суммарное отталкивание от дронов, находящихся ближе, чем safety_radius.
        - Новый вектор = current_velocity + attraction + 4 * repulsion,
        затем ограничивается по ускорению и по максимальной скорости.
    :param target_point: Целевая координата
    :type target_point: np.ndarray

    :return: numpy-массив [vx, vy]
    :rtype: np.ndarray
    """
    local_pos = state_vector[0:2]
    current_velocity = state_vector[3:5] 
    # Attraction force: единичный вектор от текущей позиции к swarm_goal
    direction = target_point - local_pos
    norm_dir = np.linalg.norm(direction)
    if norm_dir != 0:
        attraction_force = direction / norm_dir
    else:
        attraction_force = np.zeros(2)
    print(attraction_force)
    # Repulsion force: суммируем вклад от каждого дрона, если расстояние меньше safety_radius
    repulsion_force = np.zeros(2)
    # Вектор выведения дрона из равновесия (при стабилизации на границе дрона)
    unstable_vector = np.zeros(2)
    for state in env.values():
        if len(state.data) >= 3:
            xyz = state.data[1:4]
            speed = state.data[4:7]
            other_pos = np.array(xyz[0:2], dtype=float)
            distance_vector = local_pos - other_pos
            distance = np.linalg.norm(distance_vector)
            if 0 < distance < safety_radius:
                repulsion_force += distance_vector / (distance ** 2)
                print(f"+ repulsion_force = {repulsion_force}")
            direction_to_other_drone = np.linalg.norm(xyz[0:2] - state_vector[0:2])
            if (direction_to_other_drone < safety_radius + 0.1 and
                np.allclose(np.linalg.norm(speed[0:2]), 0, atol=0.1) and
                    np.linalg.norm(direction) > safety_radius + 0.2):
                unstable_vector += vector_rotation2(normalization(direction, 0.3), -np.pi / 2)
                print(f"+ unstable_vector = {unstable_vector}")
    # Вычисляем новый вектор скорости (базовый алгоритм)
    new_velocity = current_velocity + attraction_force + 4 * repulsion_force + unstable_vector + np.random.random(2)
    # Ограничиваем изменение (акселерацию) до max_acceleration
    new_velocity = limit_acceleration(current_velocity, new_velocity, max_acceleration=0.1)
    # Ограничиваем скорость до max_speed
    new_velocity = limit_speed(new_velocity, max_speed)
    return new_velocity





