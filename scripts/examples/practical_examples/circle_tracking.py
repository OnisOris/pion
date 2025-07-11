from pion import Pion
import time
import numpy as np

if __name__ == "__main__":
    num_points = 10
    radius = 1
    z_height = 1
    max_speed = 0.4
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=True)
    # Координаты x, y
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    z = np.full_like(x, z_height)
    # Расчет времени с ограничением скорости
    circumference = 2 * np.pi * radius  # Длина окружности
    min_time = circumference / max_speed  # Минимальное время для полного круга

    # Равномерное распределение времени с учетом ограничения скорости
    t = np.linspace(0, min_time, num_points)

    # Комбинируем все координаты
    points = np.column_stack((x, y, z, t))
    print(points)
    drone = Pion(
        ip=f"10.1.100.134",
        mavlink_port=5656,
        logger=True,
        dt=0.0,
        count_of_checking_points=5,
    )
    drone.arm()
    drone.takeoff()
    time.sleep(15)
    drone.trajectory_tracking(points, wait_last_point = True)
    drone.land()
    drone.stop()