import socket
import threading
from typing import Callable, Optional, Union

import numpy as np
import numpy.typing as npt
from pymavlink import mavutil

from pionfunc.annotation import Array6


def get_unique_instance_id(ip: str, instance_number=None) -> int:
    """
    Формирование id из номера устройства ip, либо из его hash, если точек не 3.
    """
    octet = ip.split(".")[-1] if ip.count(".") == 3 else str(hash(ip) % 1000)
    return int(f"{octet}{instance_number}") if instance_number else int(octet)


def get_numeric_id(unique_id: int) -> int:
    """
    Функция для сокращения максимальной длинны id
    """
    return abs(unique_id) % (10**12)


def extract_ip_id(ip: str) -> int:
    """
    Возвращает последний октет IP как строку.

    Если не получается, возвращает хэш в диапазоне [0, 1000).
    """
    parts = ip.split(".")
    if len(parts) == 4:
        try:
            return int(parts[-1])
        except ValueError:
            pass
    print(f"Ip = {ip}, \n Extract ip td = {str(abs(hash(ip)) % 1000)}")
    return abs(hash(ip)) % 1000


def start_threading(function: Callable, *args) -> threading.Thread:
    """
    Функция принимает ссылку на функцию и запускает поток

    :param function: Ссылка на функцию
    :type function: Callable
    :return: Запущенный поток
    :rtype: threading.Thread
    """
    thread = threading.Thread(target=function, args=args)
    thread.start()
    return thread


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
    except Exception("local ip error") as e:
        local_ip = "127.0.0.1"
        print(e)
    finally:
        s.close()
    return local_ip


def normalization(vector: np.ndarray, length: float = 1.0) -> np.ndarray:
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
    matrix_rotation = np.array(
        [[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]
    )
    new_vector = matrix_rotation.dot(vector)
    return new_vector


def create_connection(
    connection_method: str, address: str, port_or_baudrate: Union[int, str]
) -> mavutil.mavfile:
    """
    Создаёт MAVLink соединение.

    :param connection_method: Метод соединения, например, 'udp', 'tcp', 'serial', или другой.
    :type connection_method: str

    :param address: IP-адрес для соединения, например '127.0.0.1'
    :type address: str

    :param port_or_baudrate: Порт для соединения. Может быть целым числом или строкой.
    :type port_or_baudrate: Union[int, str]

    :return: Возвращает объект mav_socket, который представляет MAVLink соединение.
    :rtype: mavutil.mavfile
    """

    if connection_method.lower() == "serial":
        # Для Serial: device=ip, baud=port
        mav_socket = mavutil.mavlink_connection(
            device=address, baud=int(port_or_baudrate), autoreconnect=True
        )
    else:
        # Для UDP/TCP и других методов
        conn_str = f"{connection_method}:{address}:{port_or_baudrate}"
        mav_socket = mavutil.mavlink_connection(conn_str)
    return mav_socket


def update_array(
    arr: Union[list, npt.NDArray[Union[float, list, npt.NDArray[np.float64]]]],
    new_value: Union[float, list, npt.NDArray[np.float64]],
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
    vector: Union[list, npt.NDArray[np.float64]], new_value: float
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
    matrix: Union[list, npt.NDArray[np.float64]], atol: float = 8e-2
) -> bool:
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
    return np.all(
        [np.allclose(first_row, row, atol=atol) for row in matrix[1:]]
    )


def vector_reached(
    target_vector: Union[list, npt.NDArray[np.float64]],
    current_point_matrix: Union[list, npt.NDArray[np.ndarray]],
    accuracy: Union[int, float] = 5e-2,
) -> bool:
    """
    Функция сравнивает текующую позицию с целевой позицией, возвращает True в пределах погрешности accuracy.

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


def scalar_reached(
    target_vector: float,
    current_point_matrix: npt.NDArray[np.ndarray],
    accuracy: Union[int, float] = 5e-2,
) -> bool:
    """
    Функция сравнивает текующую позицию с целевой позицией, возвращает True в пределах погрешности accuracy.

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


def limit_acceleration(
    current_velocity: np.ndarray,
    target_velocity: np.ndarray,
    max_acceleration: float,
) -> np.ndarray:
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


def saturation(vector: np.ndarray, max_value: float) -> np.ndarray:
    """
    Ограничение вектора по максимальной длине.

    :param vector: Ограничеваемый вектор
    :type vector: np.ndarray

    :param max_value: Максимальное значение
    :type max_value: float

    :return: np.ndarray
    :rtype: np.ndarray
    """
    norm = np.linalg.norm(vector)
    if norm > max_value:
        return (vector / norm) * max_value
    return vector


def limit_speed0345(velocity: np.ndarray, max_speed: float) -> np.ndarray:
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


def compute_swarm_velocity(
    state_vector: Array6,
    env: dict,
    target_point: np.ndarray = np.array([0, 0]),
    params: Optional[dict] = None,
) -> np.ndarray:
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
    if params is None:
        params = {
            "attraction_weight": 1.0,
            "cohesion_weight": 1.0,
            "alignment_weight": 1.0,
            "repulsion_weight": 4.0,
            "unstable_weight": 1.0,
            "noise_weight": 1.0,
            "safety_radius": 1.0,
            "max_acceleration": 1,
            "max_speed": 0.4,
        }
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
            if 0 < distance < params["safety_radius"]:
                repulsion_force += distance_vector / (distance**2)
                print(f"+ repulsion_force = {repulsion_force}")
            direction_to_other_drone = np.linalg.norm(
                xyz[0:2] - state_vector[0:2]
            )
            if (
                direction_to_other_drone < params["safety_radius"] + 0.1
                and np.allclose(np.linalg.norm(speed[0:2]), 0, atol=0.1)
                and norm_dir > params["safety_radius"] + 0.2
            ):
                unstable_vector += vector_rotation2(
                    normalization(direction, 0.3), -np.pi / 2
                )
                print(f"+ unstable_vector = {unstable_vector}")
    # Вычисляем новый вектор скорости (базовый алгоритм)
    new_velocity = (
        current_velocity
        + params["attraction_weight"] * attraction_force
        + params["repulsion_weight"] * repulsion_force
        + params["unstable_weight"] * unstable_vector
    )
    # Ограничиваем изменение (акселерацию) до max_acceleration
    new_velocity = limit_acceleration(
        current_velocity,
        new_velocity,
        max_acceleration=params["max_acceleration"],
    )
    # Ограничиваем скорость до max_speed
    new_velocity = saturation(new_velocity, params["max_speed"])
    return new_velocity


def compute_swarm_velocity_pid(
    state_vector: Array6,
    env: dict,
    target_point: np.ndarray = np.array([0, 0]),
    params: Optional[dict] = None,
) -> np.ndarray:
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
    if params is None:
        params = {
            "attraction_weight": 1.0,
            "cohesion_weight": 1.0,
            "alignment_weight": 1.0,
            "repulsion_weight": 4.0,
            "unstable_weight": 1.0,
            "noise_weight": 1.0,
            "safety_radius": 1.0,
            "max_acceleration": 1,
            "max_speed": 0.4,
        }
    local_pos = state_vector[0:2]
    current_velocity = state_vector[3:5]
    # Attraction force: единичный вектор от текущей позиции к swarm_goal
    direction = target_point - local_pos
    norm_dir = np.linalg.norm(direction)
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
            if 0 < distance < params["safety_radius"]:
                repulsion_force += normalization(distance_vector, 1) / (
                    (distance + 1 - params["safety_radius"]) ** 2
                )
                print(f"+ repulsion_force = {repulsion_force}")
            direction_to_other_drone = np.linalg.norm(
                xyz[0:2] - state_vector[0:2]
            )
            if (
                direction_to_other_drone < params["safety_radius"] + 0.1
                and np.allclose(np.linalg.norm(speed[0:2]), 0, atol=0.1)
                and norm_dir > params["safety_radius"] + 0.2
            ):
                unstable_vector += vector_rotation2(
                    normalization(direction, 0.3), -np.pi / 2
                )
                print(f"+ unstable_vector = {unstable_vector}")
    # Вычисляем новый вектор скорости (базовый алгоритм)
    new_velocity = (
        current_velocity
        + params["repulsion_weight"] * repulsion_force
        + params["unstable_weight"] * unstable_vector
    )
    # Ограничиваем изменение (акселерацию) до max_acceleration
    new_velocity = limit_acceleration(
        current_velocity,
        new_velocity,
        max_acceleration=params["max_acceleration"],
    )
    # Ограничиваем скорость до max_speed
    new_velocity = saturation(new_velocity, params["max_speed"])
    return new_velocity


def compute_swarm_velocity_boids(
    state_vector: np.ndarray,
    env: dict,
    target_point: np.ndarray = np.array([0, 0]),
    params: Optional[dict] = None,
) -> np.ndarray:
    """
    Вычисляет желаемый вектор скорости для локального дрона с использованием правил boids.

    Алгоритм учитывает:
      - Attraction к целевой точке (target_point),
      - Cohesion: стремление к центру масс соседей,
      - Alignment: выравнивание с соседними дронами,
      - Repulsion: отталкивание от слишком близких дронов,
      - Дополнительный unstable_vector (если дрон находится в нестабильном положении),
      - Случайный шум.

    Все веса и ограничения задаются через словарь params.

    :param state_vector: вектор состояния дрона (например, [x, y, ?, vx, vy, ?])
    :param env: словарь состояний других дронов, где state.data содержит позицию и скорость
    :param target_point: целевая координата, к которой направлен дрон
    :param params: словарь с параметрами:
         - "attraction_weight": вес силы притяжения к target_point,
         - "cohesion_weight": вес силы коэф. сближения (центр масс соседей),
         - "alignment_weight": вес силы выравнивания (средняя скорость соседей),
         - "repulsion_weight": вес силы отталкивания,
         - "unstable_weight": вес дополнительного корректирующего вектора,
         - "noise_weight": вес случайного шума,
         - "safety_radius": радиус, в пределах которого действует repulsion,
         - "max_acceleration": максимальное изменение скорости,
         - "max_speed": максимальная скорость.
    :return: numpy-массив [vx, vy] – новый вектор скорости
    """
    # Если не передан словарь параметров, используем значения по умолчанию
    if params is None:
        params = {
            "attraction_weight": 1.0,
            "cohesion_weight": 1.0,
            "alignment_weight": 1.0,
            "repulsion_weight": 4.0,
            "unstable_weight": 1.0,
            "noise_weight": 1.0,
            "safety_radius": 1.0,
            "max_acceleration": 0.1,
            "max_speed": 0.4,
        }

    local_pos = state_vector[0:2]
    current_velocity = state_vector[3:5]

    # 1. Attraction: от текущей позиции к целевой точке
    direction_to_target = target_point - local_pos
    norm_dir = np.linalg.norm(direction_to_target)
    if norm_dir != 0:
        attraction_force = direction_to_target / norm_dir
    else:
        attraction_force = np.zeros(2)

    # 2. Cohesion: стремление к центру масс соседей
    neighbor_positions = []
    neighbor_velocities = []
    repulsion_force = np.zeros(2)
    unstable_vector = np.zeros(2)

    for state in env.values():
        # Предполагаем, что state.data содержит позицию (индексы 1:4) и скорость (индексы 4:7)
        if len(state.data) >= 7:
            # Позиция соседа (возьмём только x и y)
            other_pos = np.array(state.data[1:3], dtype=float)
            neighbor_positions.append(other_pos)
            # Скорость соседа
            other_vel = np.array(state.data[4:6], dtype=float)
            neighbor_velocities.append(other_vel)

            # Repulsion: если расстояние меньше safety_radius
            distance_vector = local_pos - other_pos
            distance = np.linalg.norm(distance_vector)
            if 0 < distance < params["safety_radius"]:
                repulsion_force += distance_vector / (distance**2)

            # Пример дополнительного unstable_vector, если сосед неподвижен и слишком близко
            if distance < (params["safety_radius"] + 0.1):
                # Если сосед неподвижен (скорость почти нулевая) и дрон движется достаточно быстро
                if np.allclose(
                    np.linalg.norm(other_vel), 0, atol=0.1
                ) and np.linalg.norm(direction_to_target) > (
                    params["safety_radius"] + 0.2
                ):
                    # Добавляем повернутый вектор от нормированного направления к цели
                    unstable_vector += vector_rotation2(
                        normalization(direction_to_target, 0.3), -np.pi / 2
                    )

    # 3. Cohesion force: если есть соседи
    if neighbor_positions:
        avg_neighbor_pos = np.mean(neighbor_positions, axis=0)
        cohesion_vector = avg_neighbor_pos - local_pos
        norm_cohesion = np.linalg.norm(cohesion_vector)
        if norm_cohesion != 0:
            cohesion_force = cohesion_vector / norm_cohesion
        else:
            cohesion_force = np.zeros(2)
    else:
        cohesion_force = np.zeros(2)

    # 4. Alignment force: стремление согласовать скорость с соседями
    if neighbor_velocities:
        avg_neighbor_vel = np.mean(neighbor_velocities, axis=0)
        alignment_vector = avg_neighbor_vel - current_velocity
        norm_alignment = np.linalg.norm(alignment_vector)
        if norm_alignment != 0:
            alignment_force = alignment_vector / norm_alignment
        else:
            alignment_force = np.zeros(2)
    else:
        alignment_force = np.zeros(2)

    # Случайный шум
    noise = np.random.random(2)

    # Комбинируем все силы с заданными весами
    new_velocity = (
        current_velocity
        + params["attraction_weight"] * attraction_force
        + params["cohesion_weight"] * cohesion_force
        + params["alignment_weight"] * alignment_force
        + params["repulsion_weight"] * repulsion_force
        + params["unstable_weight"] * unstable_vector
        + params["noise_weight"] * noise
    )

    # Ограничиваем изменение (акселерацию)
    new_velocity = limit_acceleration(
        current_velocity,
        new_velocity,
        max_acceleration=params["max_acceleration"],
    )
    # Ограничиваем скорость
    new_velocity = saturation(new_velocity, params["max_speed"])

    return new_velocity
