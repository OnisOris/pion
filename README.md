# Pion module

SDK для управления дронами pioneer base по mavlink

Обратите внимание на встроенный ПИД регулятор Pion: его коэффициенты по умолчанию настроены на параметры Copter_pos_aMax=3.0, Copter_pos_ff0=20.0,
Copter_pos_ff1=5.0,
Copter_pos_ff2=1.0, 
Copter_pos_k=1.1

# Подключение
Для подключения к дрону достаточно создать экземпляр класса Pion: в конструкторе инициализировано 
подключение к дрону. 
```python
from pion import Pion
ip = "127.0.0.1"
port = 8000
drone = Pion(ip=ip, mavlink_port=port)
```

При отключении от дрона можно создать новое подключение:
```commandline
drone.mavlink_socket.close()
drone.mavlink_socket = create_connection("udpout", ip, port)
```

# Методы класса Pion
- включение двигателей
```
arm()
``` 
---
 - выключение двигателей
```
disarm()
```
---
 - взлет
```
 takeoff()
```
---
 - посадка дрона
```
land()
```
---
- перемещение дрона в системе координат арены с управлением с микроконтроллера дрона
```
goto(x, y, z, yaw) 
```
---
- перемещение дрона в системе координат арены с управлением по скорости с внешнего устройства. Стандартные 
коэффициенты с встроенного регулятора актуальны для параметров: Copter_pos_aMax=3.0, Copter_pos_ff0=20.0,
Copter_pos_ff1=5.0,
Copter_pos_ff2=1.0, 
Copter_pos_k=1.1
```
goto_from_outside(x, y, z, accuracy) 
```
---
- задание дрону скорости которое необходимо параллельно менять для 
задания других скоростей. 
```
send_speed(vx, vy, vz, yaw_rate)
```
---
- запускает отдельный поток, который будет отсылать дрону вектор скорости из t_speed

```
set_v()
``` 

---

- Запуск записи траектории и скоростей дрона во времени
```
drone.check_attitude_flag = True
```
drone.trajectory - поле, содержащее в себе траекторию - матрицу 12xn, где каждая строка -
это [x, y, z, yaw, vx, vy, vz, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t], где

- x, y, z, yaw - координаты дрона,
- vx, vy, vz, v_yaw - вектор-строка скорости дрона
- v_xc, v_yc, v_zc, v_yaw_c - управляющий, задающий скорость вектор 
- t - время

В итоге матрица trajectory представляет собой вектора состояний дрона во времени attitude.

# Примеры использования

```python
from pion import Pion
import sys
import time

args = sys.argv

number_drone = sys.argv[1]
drone = Pion(ip=f"10.1.100.{number_drone}", mavlink_port=5656)
drone.speed_flag = False
if '-c' in args:
    while True:
        print(drone.position)
        time.sleep(0)
else:
    print("---")
    drone.land()
    drone.arm()
    drone.takeoff()
    time.sleep(5)
    drone.set_v()
    drone.goto_from_outside(float(args[2]), float(args[3]), float(args[4]), float(args[5]))
    time.sleep(10)
    drone.land()
    drone.stop()
```
При запуске примера в качестве аргументов идут: номер дрона, координаты x, y, z, yaw
```commandline
python test.py [номер дрона] 1 1 1 0
```
Если необходимо протестировать получаемые с дрона координаты: поставить флаг -c
```commandline
python test.py [номер дрона] -с
```
# Класс Apion
Apion - класс наследник Pion.
Все методы остались теми же, кроме set_v_async. 

Данный метод запускает асинхронно метод v_while для 
параллельной отправки векторов скоростей в дрон"
```
set_v_async()
```

```python
from pion import Apion
import sys
import asyncio

number_drone = sys.argv[1]
drone = Apion(ip=f"10.1.100.210", mavlink_port=5656)


async def main():
    drone.arm()
    drone.takeoff()
    await asyncio.sleep(5)

    # Запуск асинхронного цикла отправки скорости
    v_while_task = asyncio.create_task(drone.set_v_async())

    # Попробуем двигать дрон
    await drone.goto_from_outside(0, 0, 0, 0)

    await asyncio.sleep(10)
    drone.land()

    # Остановим все задачи
    drone.stop()
    v_while_task.cancel()  # Отменим задачу, чтобы остановить цикл


if __name__ == "__main__":
    asyncio.run(main())
```

Дополнительные примеры с инструкцией можно найти по ссылке:
https://github.com/OnisOris/Danalysis

# Важные поля дрона и их стандартные значения

Флаг для остановки цикла отдачи вектора скорости дрону
```
speed_flag = True
```
Флаг для запуска и остановки сохранения координат
```
check_attitude_flag = False
```

Флаг для остановки цикла в _message_handler
```
message_handler_flag = True
```
Порт подключения к дрону:
```
self.mavlink_port = 5656
```
Последнее сообщение из _message_handler()
```
self._msg = None
```

Напряжение батареи дрона (Вольты)
```
self.battery_voltage = None
```
Ориентация дрона и скорости по ней вида [pitch, roll, yaw, pitch_speed, roll_speed, yaw_speed]
```
self._attitude = np.array([0, 0, 0, 0, 0, 0])
```
Позиция дрона в системе координат локуса вида [x, y, z, vx, vy, vz]
```
self._position = np.array([0, 0, 0, 0, 0, 0])
```

Список потоков
```
self.threads = []
```

Задающая скорость target speed размером (4,), -> [vx, vy, vz, v_yaw], работает при запущенном потоке v_while
```
self.t_speed = np.array([0, 0, 0, 0])
```
Период отправления следующего вектора скорости
```
self.period_send_speed = 0.05
```
Период приема всех сообщений с дрона
self.period_message_handler = 0
Информация, включающая [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t],
которая складывается в матрицу (n, 17), где n - число измерений
```
self.trajectory = np.zeros((2, 17))
```

Время создания экземпляра
```
self.t0 = time.time()
```
Максимальная скорость дрона для goto_from_outside()
```
self.max_speed = 1
```
Используется для хранения последних count_of_checking_points данных в виде [x, y, z, yaw] для верификации достижения таргетной точки
```
self.last_points = np.zeros((count_of_checking_points, 4))
```

# Класс Spion

Класс Spion позиционируется, как класс симулятора Pion. Он наследуется от класса Simulator, который
моделирует материальную точку на основе метода решения дифференциальных уравнений Рунге-Кутты 4-го порядка.
Точка - класс Point имеет возможность моделирования в 2х и 3х измерениях. 
В остальном класс имеет почти те же поля и методы, что и Pion, его можно вставлять в своих проектах и
вы сможете получить такие показатели, как координаты дрона, скорости. 

Если вас не устаивают динамические показатели дрона, измените параметры ПИД контроллеров в конструкторе класса:
pid_position_controller, pid_velocity_controller.

Метод set_v для Spion не нужен, все обновление t_speed и обработка значений в целевой скорости происходит
в _message_handler, при использовании goto, этот метод останавливает цикл в _message_handler и в главном потоке происходит
управление дроном - циклом в goto

# Примечания
Если вы склонировали репозиторий, а не установили его через pip, вам необходимо установить build-essential:
```shell
sudo apt update
sudo apt install build-essential
```

Необходимо поставить следующие пакеты:
```shell
pip install numpy matplotlib pymavlink cython 
```
Далее необходимо собрать cython модуль:
```
python setup.py build_ext --inplace
```
